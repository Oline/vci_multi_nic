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
 * File         : gmii_tx.h
 * Date         : 01/06/2012
 * Authors      : Alain Greiner
 *************************************************************************
 * This object implements a packet receiver, acting as a PHY component,
 * and respecting the GMII protocol (one byte per cycle).
 * It writes packets in a file defined by the "path" constructor argument.
 * The inter-packet gap is another constructor argument.
 *************************************************************************
 * This object has 3 constructor parameters:
 * - string   name  : module name
 * - string   path  : file pathname     
 * - uint32_t gap   : number of cycles between packets
 *************************************************************************/

#ifndef SOCLIB_CABA_GMII_RX_H
#define SOCLIB_CABA_GMII_RX_H

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
class NicRxGmii
{
    // structure constants
    const std::string   m_name;
    const uint32_t	    m_gap;
    std::ifstream m_file;

    // registers
    bool                r_fsm_gap;      // inter_packet state when true
    uint32_t            r_counter;      // cycles counter (used for both gap and plen)
    uint8_t*	        r_buffer;       // local buffer containing one packet
    uint32_t            r_plen;         // packet length (in bytes)

 

    ///////////////////////////////////////////////////////////////////
    // This function is used to convert ascii to hexa.
    ///////////////////////////////////////////////////////////////////

    uint8_t atox (uint8_t el)
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
        if(m_file)
        {
            uint32_t cpt = 0;
            uint32_t data = 0;
            std::string string;
            m_file >> r_plen >> string;
            if(m_file.eof())
            {
                printf("FIN DU FICHIER\n");
                m_file.clear();
                m_file.seekg(0, std::ios::beg);
                printf("REMISE A ZERO DU FICHIER\n");
                m_file >> r_plen >> string;
            }
            //std::cout << std::hex << string << std::endl;
            for(cpt = 0; cpt < (r_plen << 1) ; cpt++)
            {
                string[cpt] = atox(string[cpt]);
                data = (data << 4)|string[cpt];
                if((cpt%2))
                {
                    r_buffer[cpt>>1] = data;
                    data = 0;
                }
            }
        }
    }

       
public:

    /////////////
    void reset()
    {
        r_fsm_gap   = true;
        r_counter   = m_gap;
    }

    ///////////////////////////////////////////////////////////////////
    // To reach the 1 Gbyte/s throughput, this method must be called 
    // at all cycles of a 125MHz clock.
    // It is therefore written as a transition and contains a 
    // two states FSM to introduce the inter-packet waiting cycles.
    ///////////////////////////////////////////////////////////////////
    void get( bool*     dv,          // data valid
              bool*     er,          // data error
              uint8_t*  dt )         // data value
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
       // printf("valeur de dt : %x\n",*dt);
       // printf("valeur de dv : %x\n",*dv);
       // printf("valeur de er : %x\n",*er);
    } // end get()
                
    //////////////////////////////////////////////////////////////
    // constructor open the file
    //////////////////////////////////////////////////////////////
    NicRxGmii( const std::string  &name,
               const std::string  &path,
               uint32_t           gap )
    : m_name(name),
      m_gap( gap ),
      m_file(path.c_str())
    {
        r_buffer        = new uint8_t[2048];
    } // end constructor

    //////////////////
    // destructor)
    //////////////////
    ~NicRxGmii()
    {
        delete [] r_buffer;
        m_file.close();
    }

}; // end NicRxGmii

}}

#endif 

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4



