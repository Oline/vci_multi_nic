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
 *         Clement Devigne <clement.devigne@etu.upmc.fr>
 *
 * Maintainers: alain 
 */

/*************************************************************************
 * File         : nic_tx_gmii.h
 * Date         : 01/06/2012
 * Authors      : Alain Greiner
 *************************************************************************
 * This object implements a packet transmitter, acting as a PHY component,
 * and respecting the GMII protocol (one byte per cycle).
 * It writes packets in a file defined by the "path" constructor argument.
 *************************************************************************
 * This object has 3 constructor parameters:
 * - string   name    : module name
 * - string   path    : file pathname.
 * - uint32_t gap     : number of cycles between packets
 *************************************************************************/

#ifndef SOCLIB_CABA_GMII_TX_H
#define SOCLIB_CABA_GMII_TX_H

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
class NicTxGmii
{
    // structure constants
    const std::string   m_name;
    std::ofstream       m_file;

    // registers
    uint32_t            r_counter;      // cycles counter (used for both gap and plen)
    uint8_t*	        r_buffer;       // local buffer containing one packet
    

    ///////////////////////////////////////////////////////////////////
    // This function is used to write one packet to the input file
    ///////////////////////////////////////////////////////////////////
    void write_one_packet()
    {
        //assert( false and "function write_paket of GMII_TX not defined");
        if (m_file)
        {
            uint32_t cpt = 0;
            uint32_t data = 0;
            // ecrit dans le fichier (debut de ligne) la valeur de r_counter et un caractere d'espacement
            m_file << (unsigned)r_counter << ' ';
            for (cpt = 0; cpt < (r_counter - 4) ; cpt += 4) // peut etre (r_counter << 1)
            {
                data = r_buffer[cpt];

                if( (cpt+1) >= (r_counter-4) )
                {
                    m_file <<std::setfill('0')<<std::setw(2)<< std::hex << data;
                    break;
                }
                data = data | (r_buffer[cpt+1]<<8);
                
                if( (cpt+2) >= (r_counter-4) )
                {
                    m_file <<std::setfill('0')<<std::setw(4)<< std::hex << data;
                    break;
                }
                data = data | (r_buffer[cpt+2]<<16);
                
                if( (cpt+3) >= (r_counter-4) )
                {
                    m_file <<std::setfill('0')<<std::setw(6)<< std::hex << data;
                    break;
                }
                data = data | (r_buffer[cpt+3]<<24);

                /*if( (cpt+3) >= (r_counter-4))
                {
                    printf("PADDING INTO FILE !!!\n");
                    printf("CPT =%d and r_counter = %d\n",cpt,r_counter);
                    uint32_t padding = 4 - (r_counter&0x3);
                    printf("PADDING INTO FILE =%d\n",padding);
                    uint32_t mask = 0xFFFFFFFF;
                    data = data & (mask>>(padding<<3));
                    printf("DATA LAST INTO FILE =%x\n",data);
                }*/

                //ecrit dans le fichier r_buffer[cpt]
                //m_file <<std::setfill('0')<<std::setw(2)<< std::hex << (unsigned)r_buffer[cpt];
                m_file <<std::setfill('0')<<std::setw(8)<< std::hex << data;
                //std::cout <<std::hex << data;
            }
            data = r_buffer[r_counter-4];
            data = data | (r_buffer[r_counter-3]<<8);
            data = data | (r_buffer[r_counter-2]<<16);
            data = data | (r_buffer[r_counter-1]<<24);
            m_file <<std::setfill('0')<<std::setw(8)<< std::hex << data;

            // quand le packet est ecrit en entier ecrit un retour a la ligne dans le fichier
            m_file << std::dec << std::endl;
        }
    }

public:

    /////////////
    void reset()
    {
        r_counter = 0;
    }

    ///////////////////////////////////////////////////////////////////
    // To reach the 1 Gbyte/s throughput, this method must be called 
    // at all cycles of a 125MHz clock.
    ///////////////////////////////////////////////////////////////////
    void put( bool     dv,          // data valid
              bool     er,          // data error
              uint8_t  dt )         // data value
    {
        if ( not dv and (r_counter != 0) )    // end of packet
        {
            write_one_packet();
            r_counter = 0;
        }
        else if ( dv )    // running packet
        {
            r_buffer[r_counter] = dt;
            r_counter           = r_counter + 1;
        }
    } // end put()
                
    //////////////////////////////////////////////////////////////
    // constructor open the file
    //////////////////////////////////////////////////////////////
    NicTxGmii( const std::string  &name,
               const std::string  &path )
    : m_name(name),
      m_file(path.c_str(),std::ios::out)
    {
        r_buffer        = new uint8_t[2048];
    } 

    //////////////////
    // destructor
    //////////////////
    ~NicTxGmii()
    {
        delete [] r_buffer;
        m_file.close();
    }

}; // end GmiiTx

}}

#endif 

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4



