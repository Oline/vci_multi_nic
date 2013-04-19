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
 * This object implements a packet transmitter, acting as a PHY component,
 * and respecting the GMII protocol (one byte per cycle).
 * It writes packets in a file defined by the "path" constructor argument.
 *************************************************************************
 * This object has 3 constructor parameters:
 * - string   name    : module name
 * - string   path    : file pathname.
 * - uint32_t gap     : number of cycles between packets
 *************************************************************************/

#ifndef SOCLIB_CABA_TX_TAP_H
#define SOCLIB_CABA_TX_TAP_H

#include <inttypes.h>
#include <systemc>
#include <assert.h>

#include <string>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>

namespace soclib { 
namespace caba {

using namespace sc_core;

///////////////
class NicTxTap
{
    // structure constants
    const std::string   m_name;
    // std::ofstream       m_file;
    int32_t             m_tap_fd;       // File descriptor for the TAP interface
    struct ifreq        *m_tap_ifr;      // TAP interface

    // registers
    uint32_t            r_counter;      // cycles counter (used for both gap and plen)
    uint8_t*	        r_buffer;       // local buffer containing one packet
    

    ///////////////////////////////////////////////////////////////////
    // This function is used to write one packet to the input file
    ///////////////////////////////////////////////////////////////////
    void write_one_packet()
    {
        if (m_tap_fd)
            {
                write(m_tap_fd, &r_buffer, r_counter);

                // uint32_t cpt = 0;
                // uint32_t data = 0;
                // write in the file r_counter value
                // m_file << (unsigned)r_counter << ' ';
                // for (cpt = 0; cpt < (r_counter - 4) ; cpt += 4)
                //     {
                //         data = r_buffer[cpt];

                //         if ((cpt+1) >= (r_counter-4))
                //             {
                //                 m_file <<std::setfill('0')<<std::setw(2)<< std::hex << data;
                //                 break;
                //             }
                //         data = data | (r_buffer[cpt+1]<<8);
                
                //         if ((cpt+2) >= (r_counter-4))
                //             {
                //                 m_file <<std::setfill('0')<<std::setw(4)<< std::hex << data;
                //                 break;
                //             }
                //         data = data | (r_buffer[cpt+2]<<16);
                
                //         if ((cpt+3) >= (r_counter-4))
                //             {
                //                 m_file <<std::setfill('0')<<std::setw(6)<< std::hex << data;
                //                 break;
                //             }
                //         data = data | (r_buffer[cpt+3] << 24);

                //         //write data from r_buffer[cpt] in the file
                //         m_file <<std::setfill('0')<<std::setw(8)<< std::hex << data;
                //     }
                // data = r_buffer[r_counter-4];
                // data = data | (r_buffer[r_counter-3]<<8);
                // data = data | (r_buffer[r_counter-2]<<16);
                // data = data | (r_buffer[r_counter-1]<<24);
                // m_file <<std::setfill('0')<<std::setw(8)<< std::hex << data;
                // m_file << std::dec << std::endl;
            }
    }

public:

    ///////////////////////////////////////////////////////////////////
    // This function is used to set the value of the TAP file descriptor
    ///////////////////////////////////////////////////////////////////
    void set_fd(int     fd)
    {
        m_tap_fd = fd;
    }

    ///////////////////////////////////////////////////////////////////
    // This function is used to set the value of the TAP file descriptor
    ///////////////////////////////////////////////////////////////////
    void set_ifr(struct ifreq     *ifr)
    {
        m_tap_ifr = ifr;
    }

    /////////////
    void reset()
    {
#ifdef SOCLIB_NIC_DEBUG
        printf("[NIC][NicTxTap][%s] resetting\n", __func__);
#endif
        r_counter = 0;
        memset(r_buffer,0,2048);
    }

    ///////////////////////////////////////////////////////////////////
    // To reach the 1 Gbyte/s throughput, this method must be called 
    // at all cycles of a 125MHz clock.
    ///////////////////////////////////////////////////////////////////
    void put( bool     dv,          // data valid
              bool     er,          // data error
              uint8_t  dt)         // data value
    {
        if (not dv and (r_counter != 0))    // end of packet
            {
                write_one_packet();
                r_counter = 0;
            }
        else if (dv)    // start or running packet
            {
                r_buffer[r_counter] = dt;
                r_counter           = r_counter + 1;
            }
    } // end put()
                
    //////////////////////////////////////////////////////////////
    // constructor open the file
    //////////////////////////////////////////////////////////////
    NicTxTap( const std::string  &name,
              const std::string  &path)
        : m_name(name)
    {
#ifdef SOCLIB_NIC_DEBUG
    printf("[NIC][%s] Entering constructor\n", __func__);
#endif
        r_buffer        = new uint8_t[2048];
    } 

    //////////////////
    // destructor
    //////////////////
    ~NicTxTap()
    {
        delete [] r_buffer;
    }

}; // end NicTxTap

}}

#endif 

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4



