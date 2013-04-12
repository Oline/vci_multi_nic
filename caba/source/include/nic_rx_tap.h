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
 *         Sylvain Leroy <sylvain.leroy@lip6.fr>
 *
 * Maintainers: sylvain
 */

/*************************************************************************
 * This object implements a packet receiver, acting as a PHY component.
 * It reads packets from a TAP interface named by the "ifname" constructor argument.
 *************************************************************************
 * This object has 3 constructor parameters:
 * - string   name    : module name
 * - uint32_t gap     : number of cycles between packets
 * - string   ifname  : TAP interface name (optionnal)
 *************************************************************************/

#ifndef SOCLIB_CABA_NIC_RX_TAP_H
#define SOCLIB_CABA_NIC_RX_TAP_H

#include <inttypes.h>
#include <systemc>
#include <assert.h>

#include <string>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>


namespace soclib {
namespace caba {

using namespace sc_core;

////////////////
class NicRxTap
{
    // structure constants
    const std::string   m_name;
    const uint32_t	    m_gap;
    std::ifstream       m_path;

    // registers
    bool                r_fsm_gap;      // inter_packet state when true
    uint32_t            r_counter;      // cycles counter (used for both gap and plen)
    uint8_t*	        r_buffer;       // local buffer containing one packet
    uint8_t*	        r_buffer_tmp;       // local buffer containing one packet
    uint32_t            r_plen;         // packet length (in bytes)

    // members
    uint32_t            m_tap_fd;       // File descriptor for the TAP interface
    // struct ifreq        m_tap_ifr;      // TAP interface


    ///////////////////////////////////////////////////////////////////
    // This function is used to convert ascii to hexa.
    ///////////////////////////////////////////////////////////////////

    uint8_t atox(uint8_t el)
    {
        if((el >= 48) and (el <= 57))
            return (el - 48);
        else if ((el >= 65) and (el <= 70))
            return ((el - 65)+10);
        else if((el >= 97) and (el <= 102))
            return ((el-97)+10);
        else
            return -1;
    }

    ///////////////////////////////////////////////////////////////////
    // This function is used to read one packet from the input file
    // It must update the r_buffer and the r_plen variables.
    ///////////////////////////////////////////////////////////////////
    void read_one_packet()
    {
        // if (m_ifp)
        // {
        //         uint32_t cpt = 0;
        //         uint32_t data = 0;
        //         uint32_t nb_words = 0;
        //         uint32_t nb_bytes_available;
        //         //string contains a packet
        //         std::string string;
        //         uint32_t i = 0;

        //         m_ifp >> r_plen >> string;
        //         nb_bytes_available = r_plen-4;

        //         // check end of file and restart it
        //         if (m_ifp.eof())
        //         {
        //                 m_ifp.clear();
        //                 m_ifp.seekg(0, std::ios::beg);
        //                 m_ifp >> r_plen >> string;
        //         }
        //         // convert all the char in string into a hexa
        //         for (cpt = 0; cpt < (r_plen << 1) ; cpt++)
        //         {
        //             string[cpt] = atox(string[cpt]);
        //             data = (data << 4)|string[cpt];
        //             if(cpt%2)
        //             {
        //                 r_buffer_tmp[cpt>>1]    = data;
        //                 data = 0;
        //             }
        //         }
        //         cpt = 0;


        //         while (nb_bytes_available)
        //         {
        //             if (nb_bytes_available > 3)
        //                 i = ((nb_words + 1)<<2) - 1;
        //             else
        //                 i = ((nb_words + 1)<<2) - (4 - nb_bytes_available) - 1;
        //             while ( i >= (nb_words << 2) )
        //             {
        //                 r_buffer[i] = r_buffer_tmp[cpt];
        //                 nb_bytes_available -- ;
        //                 cpt ++ ;
        //                 if ( i%4 == 0 )
        //                 {
        //                     nb_words ++ ;
        //                     break;
        //                 }
        //                 else i--;
        //             }
        //         }
        //         r_buffer[r_plen-4] = r_buffer_tmp[r_plen-1];
        //         r_buffer[r_plen-3] = r_buffer_tmp[r_plen-2];
        //         r_buffer[r_plen-2] = r_buffer_tmp[r_plen-3];
        //         r_buffer[r_plen-1] = r_buffer_tmp[r_plen-4];
        // }
    }

public:

    /////////////
    void reset()
    {
        r_fsm_gap   = true;
        r_counter   = m_gap;
        memset(r_buffer,0,2048);
        memset(r_buffer_tmp,0,2048);
    }

    ///////////////////////////////////////////////////////////////////
    // To reach the 1 Gbyte/s throughput, this method must be called
    // at all cycles of a 125MHz clock.
    // It is therefore written as a transition and contains a
    // two states FSM to introduce the Inter-Frame Gap waiting cycles.
    ///////////////////////////////////////////////////////////////////
    void get( bool*     dv,         // data valid
              bool*     er,         // data error
              uint8_t*  dt  )        // data value
    //bool      on)         // power enable
    {
        if ( r_fsm_gap )    // inter-packet state
            {
                *dv = false;
                *er = false;
                *dt = 0;

                r_counter = r_counter - 1;

                if (r_counter == 0 ) // end of gap
                    {
                        r_fsm_gap = false;
                        read_one_packet();
                    }
            }
        else    // running packet
            {
                *dv = true;
                *er = false;
                *dt = r_buffer[r_counter];

                r_counter = r_counter + 1;

                if ( r_counter == r_plen ) // end of packet
                    {
                        r_counter   = m_gap;
                        r_fsm_gap   = true;
                    }
            }
    } // end get()

    //////////////////////////////////////////////////////////////
    // constructors
    //////////////////////////////////////////////////////////////

    // Create the TAP interface with name specified by the kernel
    NicRxTap( const std::string  &name,
              uint32_t           gap)
        : m_name(name),
          m_gap(gap)
    {

        //         m_tap_fd = open("/dev/net/tun", O_RDWR);

        //         if ( m_tap_fd < 0 )
        //             {
        //                 std::cerr << name << ": Unable to open /dev/net/tun" << std::endl;
        //             }
        //         else
        //             {
        //                 int flags = fcntl(_fd, F_GETFL, 0);
        //                 fcntl(m_tap_fd, F_SETFL, flags | O_NONBLOCK);

        //                 memset((void*)&m_tap_ifr, 0, sizeof(m_tap_ifr));
        //                 m_tap_ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
        //                 strncpy(m_tap_ifr.ifr_name, if_name.c_str(), IFNAMSIZ);

        //                 if (ioctl(m_tap_fd, TUNSETIFF, (void *) &m_tap_ifr) < 0)
        //                     {
        //                         close(m_tap_fd);
        //                         m_tap_fd = -1;
        //                         std::cerr << name << ": Unable to setup tap interface, check privileges."
        // #ifdef __linux__
        //                                   << " (try: sudo setcap cap_net_admin=eip ./system.x)"
        // #endif
        //                                   << std::endl;
        //                     }
        //             }

        r_buffer_tmp    = new uint8_t[2048];
        r_buffer        = new uint8_t[2048];
    } // end constructor

    // Create the TAP interface with name specified by user
    NicRxTap( const std::string  &name,
              const std::string  &path,
              uint32_t           gap )
        : m_name(name),
          m_gap(gap),
          m_path(path.c_str())
    {
        r_buffer_tmp    = new uint8_t[2048];
        r_buffer        = new uint8_t[2048];
    } // end constructor

//////////////////
// destructor
//////////////////
~NicRxTap()
{
    delete [] r_buffer;
    delete [] r_buffer_tmp;
    // m_ifp.close();
}

}; // end NicRxTap

}}

#endif /* SOCLIB_CABA_NIC_RX_TAP_H */

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4



