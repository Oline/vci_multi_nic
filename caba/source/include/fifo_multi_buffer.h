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
 *         Alain Greiner <alain.greiner@lip6.fr> July 2008
 *
 * Maintainers: alain 
 */

/*************************************************************************
 * File         : fifo_multi_buffer.h
 * Date         : 01/06/2012
 * Authors      : Alain Greiner
 *************************************************************************
 * This object implements a multi-buffer FiFO designed to be used
 * by a GMII compliant multi-channels network controller. 
 * The internal structure is not known by the writer or reader:
 * The WOK flag is true if there is at least one writable buffer.
 * The ROK flag is true if there is at least one readable packet.
 * - On the writer side, the writer does not know the packet length
 *   when it starts to write data, but signals the last word of a packet.
 *   The packet length is computed by the fifo itself. The writer can 
 *   cancel a partially written packet, if required (late checksum
 *   detection, or not enoug free buffers in the fifo to store the
 *   entire packet).
 * - On the reader side, the reader consume a full paquet after asking
 *   the packet length, and signals the last word consumption.
 *   The reader can also drop an unexpected packet in the fifo.
 *************************************************************************
 * This object has 3 constructor parameters:
 * - string   name       : fifo name
 * - uint32_t bufffers   : number of buffers.
 * - uint32_t words      : number of words per buffer.
 * Both buffers & words must be power of 2.
 *************************************************************************/

#ifndef SOCLIB_CABA_FIFO_MULTI_BUF_H
#define SOCLIB_CABA_FIFO_MULTI_BUF_H

#include <inttypes.h>
#include <systemc>
#include <assert.h>
#include "arithmetics.h"
#include "static_assert.h"

namespace soclib { 
namespace caba {

using namespace sc_core;

    // writer commands
    enum fifo_multi_wcmd_t {
        FIFO_MULTI_WCMD_NOP,       // no operation       (fifo state not modified)
        FIFO_MULTI_WCMD_WRITE,     // write one word     (fifo state modified)
        FIFO_MULTI_WCMD_LAST,      // write last word    (fifo state modified)
        FIFO_MULTI_WCMD_CLEAR,     // cancel a packet    (fifo state modified)
    } ;

    // reader commands
    enum fifo_multi_rcmd_t {
        FIFO_MULTI_RCMD_NOP,       // no operation        (fifo state not modified)
        FIFO_MULTI_RCMD_READ,      // read one word       (fifo state modified)
        FIFO_MULTI_RCMD_LAST,      // read last word      (fifo state modified) 
        FIFO_MULTI_RCMD_SKIP,      // jump to next packet (fifo state modified)
    };

using soclib::common::uint32_log2;

/////////////////////
class FifoMultiBuffer
{
    // structure constants
    const std::string   m_name;
    const uint32_t	    m_buffers;
    const uint32_t	    m_words;
    const uint32_t      m_word_mask;
    const uint32_t      m_word_shift;

    // non replicated registers
    uint32_t            r_ptw;          // word write pointer
    uint32_t            r_ptr;          // word read pointer
    uint32_t            r_sts;          // number of filled buffers
    uint32_t            r_ptw_buf_save; // index of buffer containing packet first word
    uint32_t            r_word_count;   // number of written words in a packet

    // replicated registers
    uint32_t*           r_buf;          // data                 [m_buffers*m_words]
    uint32_t*           r_plen;         // number of bytes      [m_buffers]  
    bool*               r_eop;          // end of paket         [m_buffers]

public:

    /////////////
    void reset()
    {
        r_ptr          = 0;             
        r_ptw          = 0;
        r_sts          = 32;  // Because STS checks start from the MAX buffers number value
        r_ptw_buf_save = 0;
        r_word_count   = 0;
        for ( size_t x=0 ; x<m_buffers ; x++)
        {
            r_eop[x]=0;
        }
    }

    ///////////////////////////////////////////////////////////////////////////
    // this method updates the fifo state, depending on both
    // the cmd_r and cmd_w command, and must be called at all cycles.
    ///////////////////////////////////////////////////////////////////////////
    void update( fifo_multi_wcmd_t     cmd_w,          // Write command
                 fifo_multi_rcmd_t     cmd_r,          // read command
                 uint32_t              dtin,           // data to be written
                 uint32_t              padding )       // number of invalid bytes
    {
        uint32_t ptr_buf  = r_ptr >> m_word_shift;
        uint32_t ptw_buf  = r_ptw >> m_word_shift;
        uint32_t ptw_word = r_ptw &  m_word_mask;
        uint32_t ptr_word = r_ptr &  m_word_mask;
        
        // WCMD registers update (depends only on cmd_w)
        if ( cmd_w == FIFO_MULTI_WCMD_WRITE )       // write one word 
        {
            /*printf("\n***** STATUS FIFO MULTI IN WCMD = WRITE *****\n");
            printf("fifo_multi : r_ptw = %d\n",r_ptw);
            printf("fifo_multi : r_word_count = %d\n",r_word_count);
            printf("fifo_multi : r_sts = %d\n",r_sts);
            printf("fifo_multi : ptw_word = %d\n",ptw_word);*/
            r_buf[r_ptw]               = dtin;
            r_word_count               = r_word_count + 1;
            r_ptw                      = (r_ptw + 1) % (m_buffers * m_words);
            if ( ptw_word == 0)
                r_sts = r_sts - 1; 
            /*printf("\n** STATUS FIFO MULTI after WRITE **\n");
            printf("fifo_multi : r_ptw = %d\n",r_ptw);
            printf("fifo_multi : r_word_count = %d\n",r_word_count);
            printf("fifo_multi : r_sts = %d\n",r_sts);
            printf("fifo_multi : ptw_word = %d\n\n",ptw_word);*/
        }
        else if ( cmd_w == FIFO_MULTI_WCMD_LAST )  // write last word
        {
            //printf("\n***** STATUS FIFO MULTI IN WCMD = WRITE LAST *****\n");
            r_buf[r_ptw]               = dtin;
            r_plen[r_ptw_buf_save]     = (r_word_count<<2) + 4 - padding;
            //printf("fifo_multi : plen = %x\n",r_plen[r_ptw_buf_save]);
            r_eop[ptw_buf]             = true;
            //printf("fifo_multi : ptw_buf = %x\n",ptw_buf);
            r_ptw_buf_save             = (ptw_buf + 1) % m_buffers;
            r_word_count               = 0;
            r_ptw                      = ((ptw_buf + 1) % m_buffers) * m_words;
            if ( ptw_word == 0)
                r_sts = r_sts - 1; 
            //printf("fifo_multi : r_ptw = %x\n",r_ptw);
            //printf("fifo_multi : r_word_count = %x\n",r_word_count);
            //printf("fifo_multi : r_sts = %x\n",r_sts);
            //printf("fifo_multi : padding = %x\n",padding);
            //printf("fifo_multi : ptw_word = %x\n",ptw_word);
        }
        else if ( cmd_w == FIFO_MULTI_WCMD_CLEAR ) // clear the current packet
        {
            /*printf("\n***** STATUS FIFO MULTI IN WCMD = WCLEAR *****\n");
            printf("r_sts : %d\n",r_sts);
            printf("r_ptw : %d\n",r_ptw);
            printf("ptw_buf : %d\n",ptw_buf);
            printf("r_word_count : %d\n",r_word_count);*/
            uint32_t first_buf = r_ptw_buf_save;
            //printf("first buf : %d\n",first_buf);
            uint32_t nbuf;
            if ( ptw_buf >= first_buf ) nbuf = ptw_buf - first_buf + 1;
            else                        nbuf = (ptw_buf + m_buffers) - first_buf + 1;
            //printf("nbuf : %d\n",nbuf);
            r_ptw                      = r_ptw_buf_save * m_words;
            r_word_count               = 0;
            r_sts                      = r_sts + nbuf ;
            /*printf("r_sts : %d\n",r_sts);
            printf("r_ptw : %d\n",r_ptw);
            printf("ptw_buf : %d\n",ptw_buf);
            printf("r_word_count : %d\n\n",r_word_count);*/
        }

        // RCMD registers update (depends only on cmd_r)
        if ( cmd_r == FIFO_MULTI_RCMD_READ )    // read one word
        {
            r_ptr                                = (r_ptr + 1) % (m_buffers * m_words);
            if ( ptr_word == (m_words-1) ) r_sts = r_sts + 1;
        }
        else if ( cmd_r == FIFO_MULTI_RCMD_LAST ) // read last word
        {
            r_eop[ptr_buf]    = false;
            r_ptr             = ((ptr_buf + 1) % m_buffers) * m_words;
            r_sts             = r_sts + 1;
        }
        else if ( cmd_r == FIFO_MULTI_RCMD_SKIP ) // skip one packet
        {
            //printf("ptr_buf = %d\n",ptr_buf);
            //printf("r_sts before skip = %d\n",r_sts);
            //printf("r_ptr = %d\n",r_ptr);
            uint32_t plen     = r_plen[ptr_buf];
            //printf("plen = %d\n",plen);
            uint32_t last_ptr = (r_ptr + ((plen - 4)>>2)) % (m_buffers * m_words);
            //printf("last_ptr = %d\n",last_ptr);
            uint32_t last_buf = last_ptr >> m_word_shift;
            //printf("last_buf = %d\n",last_buf);
            r_eop[last_buf]   = false;
            //printf("r_eop is : %d\n",r_eop[last_buf]);
            r_ptr             = ((last_buf + 1) % m_buffers) * m_words;
            //printf("r_ptr = %d\n",r_ptr);
            r_sts             = r_sts + 1 + ((plen>>2) / m_words);
            //printf("r_sts = %d\n",r_sts);
        }
    } // end update()

    //////////////////////////////////////////////////////////////
    // This method returns the pointed word.
    // It does not modify the fifo state
    //////////////////////////////////////////////////////////////
    uint32_t data()
    { 
        return r_buf[r_ptr];
    }

    //////////////////////////////////////////////////////////////
    // This method returns the current packet length.
    // It must be used before reading the data.
    // It does not modify the fifo state
    //////////////////////////////////////////////////////////////
    uint32_t plen()
    { 
        return r_plen[r_ptr>>m_word_shift];
    }

    //////////////////////////////////////////////////////////////
    // This method returns true if there is a completed packet. 
    // It does not modify the fifo state
    //////////////////////////////////////////////////////////////
    bool rok()  
    {
        for ( size_t x=0 ; x<m_buffers ; x++)
        {
            if ( r_eop[x] ) 
            {
                //printf("le buff du reop est : %d\n",x);
                return true; 
            }
        }
        return false;
    }

    //////////////////////////////////////////////////////////////
    // This method returns true if there is a free buffer.
    // It does not modify the fifo state
    //////////////////////////////////////////////////////////////
    bool wok()
    {
        return (r_sts > 0);
    }

    //////////////////////////////////////////////////////////////
    // constructor checks parameters, allocates the memory
    // and computes various constants
    //////////////////////////////////////////////////////////////
    FifoMultiBuffer( const std::string  &name,
                     uint32_t           buffers,
                     uint32_t           words )
    : m_name(name),
      m_buffers(buffers),
      m_words(words),
      m_word_mask(words-1),
      m_word_shift(uint32_log2(words))
    {
        assert(IS_POW_OF_2(buffers));
        assert(IS_POW_OF_2(words));

        r_buf        	= new uint32_t[buffers*words];
        r_eop           = new bool[buffers];
        r_plen          = new uint32_t[buffers];

    } // end constructor

    //////////////////
    // destructor)
    //////////////////
    ~FifoMultiBuffer()
    {
        delete [] r_buf;
        delete [] r_eop;
        delete [] r_plen;
    }

}; // end FifoMultiBuffer

}}

#endif 

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4



