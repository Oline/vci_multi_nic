/* -*- c++ -*-
 *
 * SOCLIB_LGPL_HEADER_BEGIN
 *
 * This file is part of SoCLib, GNU LGPLv2.1.
 *
 * SoCLib is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; version 2.1 of the License.
 *
 * SoCLib is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with SoCLib; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * SOCLIB_LGPL_HEADER_END
 *
 * Copyright (c) UPMC, Lip6
 *         Alain Greiner <alain.greiner@lip6.fr> July 2012
 *         Clement Devigne <clement.devigne@etu.upmc.fr>
 *         Sylvain Leroy <sylvain.leroy@lip6.fr>
 *         
 *
 * Maintainers: alain 
 */

/**********************************************************************
 * File         : nic_rx_channel.h
 * Date         : 01/06/2012
 * Authors      : Alain Greiner
 **********************************************************************
 * This object implements an hardware communication channel,
 * to store received packets (from NIC to software), 
 * for a multi-channels, GMII compliant, network controller.
 * It contains two  Kbytes containers, and is acting as
 * a two slots fifo (one container = one slot).
 *
 * The container descriptor is defined in the first 32 words :
 * word0:       | NB_WORDS          | NB_PACKETS        |
 * word1:       | PLEN[1]           | PLEN[0]           |
 *              | ...               | ...               |
 * word31:      | PLEN[61]          | PLEN[62]          |
 * The packets are stored in the 1024-32) following words,
 * and the packets are word-aligned.
 *
 * The number of containers is not known by the writer or reader:
 * The WOK flag is true if there is a writable container.
 * The ROK flag is true if there is a readable container.
 * Both container and word addressing is handled by the channel itself.
 * - On the writer side, the NIC writes one or several packets
 *   in an open container, after checking - for each packet -
 *   that there is enough free space in the container.
 *   It closes the container if there is not enough space.
 *   The container descriptor (first 32 words) is automaically
 *   filled by the channel itself.
 *   An internal timer defines the maximum waiting time for
 *   a partially filled container: This timer is reset each time
 *   a new (start of) packet is registered in the container.
 *   When the timer fires, the container is automatically closed.
 * - On the reader side, the software read a full container,
 *   and releases the container when this is done.
 *   As the container can be partially filled, the software can get
 *   the actual number of useful words, before transfering the data.
 **********************************************************************
 * This object has 1 constructor parameter:
 * - string   name               : channel name
 **********************************************************************/

#ifndef SOCLIB_CABA_NIC_RX_CHANNEL
#define SOCLIB_CABA_NIC_RX_CHANNEL

#include <inttypes.h>
#include <systemc>
#include <assert.h>

namespace soclib { 
namespace caba {

using namespace sc_core;

#define NIC_CONTAINER_SIZE          1024 // Size in uint32_t
#define NIC_CONTAINER_SIZE_BYTES    NIC_CONTAINER_SIZE*4
#define MAX_PACKET                  ((NIC_CONTAINER_SIZE_BYTES-4)/62)

// writer commands
enum rx_channel_wcmd_t {
    RX_CHANNEL_WCMD_NOP,       // no operation             (channel state not modified)
    RX_CHANNEL_WCMD_WRITE,     // write one word           (channel state modified) 
    RX_CHANNEL_WCMD_LAST,      // write last word          (channel state modified)
    RX_CHANNEL_WCMD_CLOSE,     // close container          (channel state modified)
};

// reader commands
enum rx_channel_rcmd_t {
    RX_CHANNEL_RCMD_NOP,       // no operation             (channel state not modified)
    RX_CHANNEL_RCMD_READ,      // read one word            (channel state modified)
    RX_CHANNEL_RCMD_RELEASE,   // release container        (channel state modified)
};

class NicRxChannel
{
    // structure constants
    const std::string   m_name;
    uint32_t      m_timeout;           // max waiting cycles

    // registers
    uint32_t            r_ptw_word;          // word write pointer (in container)
    uint32_t            r_ptw_cont;          // container write pointer
    uint32_t            r_ptr_word;          // word read pointer (in container)
    uint32_t            r_ptr_cont;          // container read pointer
    uint32_t            r_sts;               // number of filled containers
    uint32_t            r_pkt_index;         // packet index in a container
    uint32_t            r_pkt_length;        // packet length counter
    int32_t             r_timer;             // cycle counter for timeout

    // containers
    uint32_t**          r_cont;              // data[2][1024]

public:

    /////////////
    void reset()
    {
        uint32_t k;
        r_ptr_word    = 0;
        r_ptr_cont    = 0;
        r_ptw_word    = (MAX_PACKET/2)+1;
        r_ptw_cont    = 0;
        r_pkt_index   = 0;
        r_pkt_length  = 0;
        r_sts         = 0;
        r_timer       = m_timeout;
        for (k = 0; k < 2;k++)
            memset(r_cont[k], 0, NIC_CONTAINER_SIZE);
    }

    /////////////////////////////////////////////////////
    // This method updates  the internal channel state,
    // depending on both cmd_w and cmd_r commands,
    // and must be called at each cycle.
    /////////////////////////////////////////////////////
    void update( rx_channel_wcmd_t      cmd_w,      // writer command
                 rx_channel_rcmd_t      cmd_r,      // reader command
                 uint32_t               wdata,      // data to be written
                 uint32_t               padding )   // number of padding bytes
    {
        uint32_t    container_index = r_ptw_cont; // Container number/index [0,1]

        assert((r_sts <= 2)
               and "ERROR in NIX_RX_CHANNEL : STS overflow");

        // WCMD registers update (depends only on cmd_w)
        if ( cmd_w == RX_CHANNEL_WCMD_WRITE )      // write one packet word
            {
                assert( (r_ptw_word < NIC_CONTAINER_SIZE) and 
                        "ERROR in NIC_RX_CHANNEL : write pointer overflow" );

                if ( r_sts < 2 )    // at least one empty container
                    {
                        r_cont[container_index][r_ptw_word]    = wdata;
                        r_ptw_word               = r_ptw_word + 1;
                        r_pkt_length             = r_pkt_length + 4;
                    }
                else
                    {
                        assert( "ERROR in NIC_RX_CHANNEL : illegal write request" );
                    }
            }
        //////////////////////////////////////////////////////////////////////////
        else if ( cmd_w == RX_CHANNEL_WCMD_LAST )  // write last word in a packet
                                                   // and write packet length
            {
                assert( (r_ptw_word < NIC_CONTAINER_SIZE) and 
                        "ERROR in NIC_RX_CHANNEL : write pointer overflow" );

                assert( (r_pkt_index < MAX_PACKET) and
                        "ERROR in NIC_RX_CHANNEL : packet index larger than 61" );

                if ( r_sts < 2 )    // at least one container writable
                    {
                        uint32_t    plen         = r_pkt_length + 4 - padding;
                        bool        odd          = (r_pkt_index & 0x1);
                        uint32_t    word         = (r_pkt_index >> 1) + 1;

                        r_cont[container_index][r_ptw_word]    = wdata;
                        r_ptw_word               = r_ptw_word + 1;
                        if (odd) r_cont[container_index][word] = (r_cont[container_index][word] & 0x0000FFFF) | plen<<16;
                        else     r_cont[container_index][word] = (r_cont[container_index][word] & 0xFFFF0000) | plen;
                        r_pkt_index              = r_pkt_index + 1;
                        r_pkt_length             = 0;
                    }
                else
                    {
                        assert( "ERROR in NIC_RX_CHANNEL : illegal write request" );
                    }
            }
        //////////////////////////////////////////////////////////////////////////
        else
            if ( cmd_w == RX_CHANNEL_WCMD_CLOSE ) // close the current container
                {
                    r_cont[container_index][0]  = (r_ptw_word<<16) | r_pkt_index;
                    r_ptw_word                  =  ((MAX_PACKET>>1)+1);
                    r_ptw_cont                  = (r_ptw_cont + 1) % 2;
                    r_sts                       = r_sts + 1;
                    r_pkt_index                 = 0;
                    r_timer                     = m_timeout;
                }
        //////////////////////////////////////////////////////////////////////////
            else // kind of IDLE state due to command NOP
                {
                    // close current container if time-out
                    if ( r_timer <= 0 ) 
                        {
                            r_cont[container_index][0]  = (r_ptw_word<<16) | r_pkt_index;
                            r_ptw_word                  =((MAX_PACKET>>1)+1);
                            r_ptw_cont                  = (r_ptw_cont + 1) % 2;
                            r_sts                       = r_sts + 1;
                            r_pkt_index                 = 0;
                            r_timer                     = m_timeout;
                        }
                }

        // timer decrement if container not empty AND 1 byte has been writen
        if (r_ptw_word > ((MAX_PACKET>>1)+1) ) 
            {
                // decrement only if 1 Byte/Word has been writen
                r_timer = r_timer - 1;
            }

        // RCMD register update (depends only on cmd_r)
        
        if ( cmd_r == RX_CHANNEL_RCMD_READ )       // read one container word
            {
                assert( (r_ptr_word < NIC_CONTAINER_SIZE) and
                        "ERROR in NIC_RX_CHANNEL : read pointer overflow" );
                if ( r_sts > 0 )    // at least one filled container
                    {
                        r_ptr_word++;
                    }
                else
                    {
                        // /!\ Should be an ERROR HANDLING here, not an assert
                        assert( "ERROR in NIC_RX_CHANNEL : illegal read request" );
                    }
            }
        else if ( cmd_r == RX_CHANNEL_RCMD_RELEASE ) // release the current container
            {
                r_ptr_word = 0;
                memset(r_cont[r_ptr_cont], 0, NIC_CONTAINER_SIZE);
                r_ptr_cont = (r_ptr_cont + 1) % 2;
                r_sts      = r_sts - 1;
            }
        
    } // end update()

    /////////////////////////////////////////////////////////////
    // This method returns the current word value in the 
    // current container. It does not modify the channel state.
    /////////////////////////////////////////////////////////////
    uint32_t data()
    { 
        return r_cont[r_ptr_cont][r_ptr_word];
    }

    /////////////////////////////////////////////////////////////
    // This method returns the number of useful words contained 
    // in a container. It does not modify the channel state.
    /////////////////////////////////////////////////////////////
    uint32_t nwords()
    { 
        return r_cont[r_ptr_cont][0]>>16;
    }

    /////////////////////////////////////////////////////////////
    // This method returns the number of free bytes in the 
    // current container. It does not modify the channel state.
    /////////////////////////////////////////////////////////////
    uint32_t space()
    { 
        return (1024 - r_ptw_word)<<2;
    }

    /////////////////////////////////////////////////////////////
    // This method returns true if there is a full container.
    // It does not modify the channel state.
    /////////////////////////////////////////////////////////////
    bool rok()  
    {
        return (r_sts > 0);
    }

    /////////////////////////////////////////////////////////////
    // This method returns true if there is a free container.
    // It does not modify the channel state.
    /////////////////////////////////////////////////////////////
    bool wok()
    {
        return (r_sts < 2);
    }

    /////////////////////////////////////////////////////////////
    // This method returns the current value of timeout.
    // It does not modify the channel state.
    /////////////////////////////////////////////////////////////
    int32_t get_r_timer()
    {
        return r_timer;
    }
    
    /////////////////////////////////////////////////////////////
    // This method returns the value of init timeout.
    /////////////////////////////////////////////////////////////
    int32_t get_m_timeout()
    {
        return m_timeout;
    
    }
    /////////////////////////////////////////////////////////////
    // This method set a new value for init timeout.
    /////////////////////////////////////////////////////////////

    void set_timeout (uint32_t timeout)
    {
        m_timeout = timeout;
    }
    //////////////////////////////////////////////////////////////
    // constructor checks parameters and allocates the memory
    // for the containers.
    //////////////////////////////////////////////////////////////
    NicRxChannel( const std::string  &name,
                  uint32_t     timeout )
        : m_name(name),
          m_timeout(timeout)
    {
        // DISPATCH_FSM needs 1024 cycles to fill a container.
        // The FSM needs/takes 379 cycles to write the biggest packet.
        // Then to be usefull, the timeout must be bigger than 379.
        // Minimal value might be 1024 to be sure that every container can be filled up
        // with a continuous stream.
        assert((m_timeout > 379)
               and "ERROR in NIC_RX_CHANNEL : STS overflow");

        r_cont    = new uint32_t*[2];
        r_cont[0] = new uint32_t[NIC_CONTAINER_SIZE];
        r_cont[1] = new uint32_t[NIC_CONTAINER_SIZE];
    } 

    //////////////////
    // destructor
    //////////////////
    ~NicRxChannel()
    {
        delete [] r_cont[0];
        delete [] r_cont[1];
        delete [] r_cont;
    }

}; // end NicRxChannel

}}

#endif 

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4



