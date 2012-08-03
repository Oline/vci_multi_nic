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
 *         Alain Greiner <alain.greiner@lip6.fr> Juin 2012
 *         Clement Devigne <clement.devigne@etu.upmc.fr>
 */

//////////////////////////////////////////////////////////////////////////////////
//  This component is a multi-channels, GMII compliant, NIC controller.
//  This component makes the assumption that the VCI RDATA & WDATA fields
//  have 32 bits. The number of channels is a constructor parameter
//  and cannot be larger than 8.
//
//  Thic component has no DMA capability: All data transfers mut be performed
//  by software, or by an external DMA engine.
//  The data transfer unit between software and the NIC is a container.
//  A container is a 4K bytes buffer, containing an integer number
//  of ariable size packets. Packet length is between 64 to 1538 bytes.
//
//  The first 32 words of a container are the container descriptor:
//      word0:       | NB_WORDS          | NB_PACKETS        |
//      word1:       | PLEN[1]           | PLEN[0]           |
//                   | ...               | ...               |
//      word31:      | PLEN[61]          | PLEN[60]          |
//
// There is at most 62 packets in a container.
// - NB_PACKETS is the actual number of packets in the container
// - NB_WORDS is the number of useful words in the container
// - PLEN[i] is the number of bytes for packet [i].
//
// The packets are stored in the (1024-32) following words,
// and the packets are word-aligned.
//
//  In order to support various protection mechanisms, each channel takes
//  a segment of 8 Kbytes in the address space:
//  - The first 4K bytes contain the configuration and status registers
//  - The second 4K bytes define the current container.
//
//  There is two IRQ lines for each channel:
//  - RX_IRQ[k] is activated as soon as ther is at least one RX_container
//    containing data for channel (k).
//  - TX_IRQ[k] is activated as soon a there is at least one TX_container
//    empty for chnnel (k).
/////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include <cassert>
#include <cstdio>
#include "alloc_elems.h"
#include "../include/vci_multi_nic.h"
#include "../../../include/soclib/multi_nic.h"

namespace soclib { namespace caba {


// Declaration of crc table
static uint32_t crc_table[] =
{
    0x4DBDF21C, 0x500AE278, 0x76D3D2D4, 0x6B64C2B0,
    0x3B61B38C, 0x26D6A3E8, 0x000F9344, 0x1DB88320,
    0xA005713C, 0xBDB26158, 0x9B6B51F4, 0x86DC4190,
    0xD6D930AC, 0xCB6E20C8, 0xEDB71064, 0xF0000000
};


#define tmpl(t) template<typename vci_param> t VciMultiNic<vci_param>

tmpl(uint32_t)::read_register(uint32_t addr)
{
    uint32_t sel_register;
    size_t channel = (size_t)((addr & 0x0001C000) >> 14);
    size_t cell    = (size_t)((addr & 0x00001FFF) >> 2);
    //bool   burst   = (addr & 0x00002000);

    if ( p_vci.rspack.read() )
    {
                switch(cell)
                {
                    case GENERAL_CHAN_MAC_ADDR_SET :
                    {
                        sel_register = r_channel_mac_addr_set.read();
                        break;
                    }
                    case VIS :
                    {
                        sel_register = r_channel_active_channels.read();
                        //printf("sel_register in VIS READ = %x\n",sel_register);
                        break;
                    }
                    case RX_ROK :
                    {
                        sel_register = r_rx_channel[channel]->rok();
                        break;
                    }
                    case RX_PKT :
                    {
                        sel_register = r_rx_g2s_npkt.read();
                        //printf("sel_register RX_PKT = %d\n",sel_register);
                        break;
                    }
                    case RX_CRC_SUCCESS :
                    {
                        sel_register = r_rx_g2s_npkt_crc_success.read();
                        //printf("sel_register RX_CRC_SUCCESS = %d\n",sel_register);
                        break;
                    }
                    case RX_CRC_FAIL :
                    {
                        sel_register = r_rx_g2s_npkt_crc_fail.read();
                        //printf("sel_register RX_CRC_FAIL = %d\n",sel_register);
                        break;
                    }
                    case RX_ERR_MII :
                    {
                        sel_register = r_rx_g2s_npkt_err.read();
                        break;
                    }
                    case RX_MFIFO_SUCCESS :
                    {
                        sel_register = r_rx_des_npkt_write_mfifo_success.read();
                        //printf("sel_register MFIFO_SUCCESS = %d\n",sel_register);
                        break;
                    }
                    case RX_ERR_SMALL :
                    {
                        sel_register = r_rx_des_npkt_small.read();
                        break;
                    }
                    case RX_ERR_OVERFLOW :
                    {
                        sel_register = r_rx_des_npkt_overflow.read();
                        break;
                    }
                    case RX_ERR_MFIFO_FULL :
                    {
                        sel_register =r_rx_des_npkt_err_mfifo_full.read();
                        break;
                    }
                    case RX_ERR_IN_DES :
                    {
                        sel_register = r_rx_des_npkt_err_in_des.read();
                        break;
                    }
                    case RX_CHANNEL_SUCCESS :
                    {
                        sel_register = r_rx_dispatch_npkt_wchannel_success.read();
                        break;
                    }
                    case RX_CHANNEL_FAIL :
                    {
                        sel_register = r_rx_dispatch_npkt_wchannel_fail.read();
                        break;
                    }
                    case RX_MAC_ADDR_FAIL :
                    {
                        sel_register = r_rx_dispatch_npkt_skip_adrmac_fail.read();
                        //printf("sel_register MAC ADDR FAIL = %d\n",sel_register);
                        break;
                    }
                    case TX_PKT :
                    {
                        sel_register = r_tx_npkt.read();
                        break;
                    }
                    case TX_ERR_SMALL :
                    {
                        sel_register = r_tx_npkt_small.read();
                        break;
                    }
                    case TX_ERR_OVERFLOW :
                    {
                        sel_register = r_tx_npkt_overflow.read();
                        break;
                    }
                    case TX_WOK :
                    {
                        sel_register = r_tx_channel[channel]->wok();
                        break;
                    }
              }//end switch cell
    }
    return sel_register;
}

/////////////////////////
tmpl(void)::transition()
{
    if (!p_resetn) 
    {
        r_nic_on = 0;
        r_vci_fsm          = VCI_IDLE;
        r_vci_ptr          = 0;
        r_vci_ptw          = 0;
        r_channel_active_channels   = 0;
        r_channel_mac_addr_set = 0; 
        r_rx_sel_channel_wok = 0;
        r_rx_dispatch_broadcast = false;
        r_rx_sel_space_timeout_ok = 0;
        r_rx_g2s_fsm       = RX_G2S_IDLE;
        r_rx_des_fsm       = RX_DES_READ_0;
        r_rx_dispatch_fsm  = RX_DISPATCH_IDLE;
        r_tx_dispatch_fsm  = TX_DISPATCH_IDLE;
        r_tx_s2g_fsm       = TX_S2G_IDLE;
        
        r_rx_g2s_npkt = 0;
        r_rx_g2s_npkt_crc_success = 0;
        r_rx_g2s_npkt_crc_fail = 0;
        r_rx_g2s_npkt_err = 0;

        //r_rx_des_npkt_send_drop_mfifo_full = 0;
        //r_rx_des_npkt_send_drop_invplen = 0;
        //r_rx_des_counter_bytes = 0;
        r_rx_des_counter_bytes = 1;
        r_rx_des_padding = 0;
        r_rx_des_npkt_err_in_des = 0;
        r_rx_des_npkt_write_mfifo_success = 0;
        r_rx_des_npkt_err_mfifo_full = 0;
        r_rx_des_npkt_small = 0;
        r_rx_des_npkt_overflow = 0;

        r_rx_dispatch_npkt_skip_adrmac_fail = 0;
        r_rx_dispatch_npkt_wchannel_success = 0;
        r_rx_dispatch_npkt_wchannel_fail    = 0;


        
        r_tx_dispatch_first_bytes_pckt = 1;
        r_tx_s2g_checksum = 0x00000000;
        r_tx_npkt_overflow = 0;
        r_tx_npkt_small = 0;
        r_tx_npkt = 0;
        r_tx_dispatch_interne = 0;
        r_tx_dispatch_dt0 = 0;
        r_tx_dispatch_dt1 = 0;
        r_tx_dispatch_dt2 = 0;
        r_tx_dispatch_addr_mac_src_fail = 0;
        r_tx_dispatch_pipe_count = 2;
       

        r_rx_fifo_stream.init();
        r_rx_fifo_multi.reset();
        r_tx_fifo_stream.init();
        r_bp_fifo_multi.reset();

        r_gmii_rx.reset();
        r_gmii_tx.reset();

        for ( size_t k = 0 ; k < m_channels ; k++ )
        {
            r_rx_channel[k]->reset();
            r_tx_channel[k]->reset();
        }
        return;
    }

    ///////////////////////////////////////////////////////////////////////////
    // This VCI_FSM controls the VCI TARGET port
    // It is inspired from the model of the vci_simple_ram.
    // The VCI PLEN field must be multiple of 4, and the BE field is not used.
    // We acknowledge the VCI command, and decode it in the IDLE state.
    // - All configuration (write) and status (read) accesses must be one flit.
    // - The read data transfers (from RX channel container), or the
    //   write data transfers (to TX channel container) can be split
    //   into several bursts, at contiguous addresses, as the container
    //   behave as FIFOs.
    ///////////////////////////////////////////////////////////////////////////

    // default values for rx_channel and tx_channel commands
    rx_channel_rcmd_t rx_channel_rcmd  = RX_CHANNEL_RCMD_NOP;
    tx_channel_wcmd_t tx_channel_wcmd  = TX_CHANNEL_WCMD_NOP;
    uint32_t          tx_channel_wdata = 0;

    switch(r_vci_fsm.read()) 
    {
        //////////////
        case VCI_IDLE:  // decode the VCI command
        {
            if (p_vci.cmdval.read() )
            {
            //printf("VCI_IDLE\n");
                typename vci_param::addr_t	address = p_vci.address.read();
                typename vci_param::cmd_t	cmd     = p_vci.cmd.read();
                typename vci_param::plen_t  plen    = p_vci.plen.read();
//printf("address vci = %x\n",(uint32_t)address);
//printf("plen vci = %x\n",(uint32_t)plen);
                assert ( ((plen & 0x3) == 0) and
                "ERROR in VCI_MULTI_NIC : PLEN field must be multiple of 4 bytes");

                assert ( (m_segment.contains(address)) and
                "ERROR in VCI_MULTI_NIC : ADDRESS is out of segment");

                r_vci_srcid	   = p_vci.srcid.read();
                r_vci_trdid	   = p_vci.trdid.read();
                r_vci_pktid	   = p_vci.pktid.read();
                r_vci_wdata    = p_vci.wdata.read();

                /*size_t channel = (size_t)((address & 0x0000E000) >> 13);
                size_t cell    = (size_t)((address & 0x00000FFF) >> 2);
                bool   burst   = (address & 0x00001000);*/

                size_t channel = (size_t)((address & 0x0001C000) >> 14);
                //size_t cell    = (size_t)((address & 0x00001FFF) >> 2);
                bool   burst   = (address & 0x00002000);
                
//printf("channel = %d\n",(uint32_t)channel);

                r_vci_channel  = channel;

                assert( (channel < m_channels) and
                "VCI_MULTI_NIC error : The channel index (ADDR[16:14] is too large");

	            if ( burst and (cmd == vci_param::CMD_WRITE) )      // TX_BURST transfer
                {
                    if ( p_vci.eop.read() ) 
                        r_vci_fsm = VCI_WRITE_TX_LAST;
                    else                           
                        r_vci_fsm = VCI_WRITE_TX_BURST;
                }
                else if ( burst  and (cmd == vci_param::CMD_READ) ) // RX_BURST transfer
                {
//printf("RX_BURST transfer\n");
                    r_vci_nwords  = (size_t)(plen >> 2);
                    r_vci_fsm     = VCI_READ_RX_BURST;
                }
                
                else if ( not burst and (cmd == vci_param::CMD_READ) )           // REG read access
                {
                    assert( p_vci.eop.read() and
                    "ERROR in VCI_MULTI_NIC : TX_WOK read access must be one flit");
                    r_vci_fsm = VCI_READ_REG;
                }
                
                else if ( not burst and (cmd == vci_param::CMD_WRITE))
                {
                    assert ( p_vci.eop.read() and
                            "ERROR in VCI_MULTI_NIC : WRITE_REG write access must be one flit");
                    r_vci_fsm = VCI_WRITE_REG;
                }
                else
                {
                    assert( false and
                    "ERROR in VCI_MULTI_NIC : illegal VCI command");
                }
            }
            break;
        }
        ///////////////////////
        case VCI_WRITE_TX_BURST: // write data[i-1] in tx_channel[k]
                                 // and check if data[i] matches write pointer
        {
            size_t channel = r_vci_channel.read();

            assert ( r_tx_channel[channel]->wok() and
            "ERROR in VCI_MULTI_NIC : tx_channel should not be full in VCI_WRITE_TX_BURST");
            if ( p_vci.cmdval.read() )
            {
                // data[i-1]
                tx_channel_wcmd  = TX_CHANNEL_WCMD_WRITE;
                tx_channel_wdata = r_vci_wdata.read();

                // data[i]
                typename vci_param::addr_t	address = p_vci.address.read();
//printf("address in write tx burst : %x\n",(uint32_t)address);
                assert( (((address & 0x00000FFF) >> 2) == r_vci_ptw.read()) and
                "ERROR in VCI_MULTI_NIC : address must be contiguous in VCI_WRITE_TX_BURST");

                r_vci_wdata      = p_vci.wdata.read();
                r_vci_ptw        = r_vci_ptw.read() + 1;
                if ( p_vci.eop.read() )
                {
                    r_vci_fsm = VCI_WRITE_TX_LAST;
                }
            }
            break;
        }
        ///////////////////////
        case VCI_WRITE_TX_LAST: // write last word of the burst in tx_channel
                                // and send VCI write response
        {
            if ( p_vci.rspack.read() )
            {
                tx_channel_wcmd  = TX_CHANNEL_WCMD_WRITE;
                tx_channel_wdata = r_vci_wdata.read();
                r_vci_fsm = VCI_IDLE;
//printf("write LAST, wdata = %x\n",tx_channel_wdata);
            }
            break;
        }
        /*
        /////////////////////
        case VCI_READ_TX_WOK:   // send tx_channnel WOK in VCI response
        {
            if ( p_vci.rspack.read() )
            {
                r_vci_fsm = VCI_IDLE;
            }
            break;
        }
        */

        ////////////////////////
       /* case VCI_WRITE_TX_CLOSE: // close tx_channel, reset r_vci_ptw pointer
                                 // and send VCI write response
        {
            if ( p_vci.rspack.read() )
            {
//printf("TX_CLOSE in VCI_FSM\n");
                tx_channel_wcmd  = TX_CHANNEL_WCMD_CLOSE;
                r_vci_ptw        = 0;
                r_vci_fsm        = VCI_IDLE;
            }
            break;
        }*/
        ///////////////////////
        case VCI_READ_RX_BURST: // check pointer and send data in VCI response
        {
            size_t channel = r_vci_channel.read();

            assert ( r_rx_channel[channel]->rok() and
            "ERROR in VCI_MULTI_NIC : rx_channel should not be empty in VCI_READ_RX_BURST");

            if ( p_vci.rspack.read() )
            {
                rx_channel_rcmd  = RX_CHANNEL_RCMD_READ;
                r_vci_nwords     = r_vci_nwords.read() - 1;
                r_vci_ptr        = r_vci_ptr.read() + 1;
                if ( r_vci_nwords.read() == 1 ) r_vci_fsm = VCI_IDLE;
            }
            break;
        }
        /////////////////////
        case VCI_WRITE_REG :
        {
            typename vci_param::addr_t	address = p_vci.address.read();
            size_t channel = r_vci_channel.read();
            size_t cell    = (size_t)((address & 0x00001FFF) >> 2);

            if ( p_vci.rspack.read() )
            {
                switch(cell)
                {
                    case NIC_ON :
                    {
                        r_nic_on = r_vci_wdata.read();
                        break;
                    }
                    case TX_CLOSE :
                    {
                        printf("TX_CLOSE\n");
                        tx_channel_wcmd = TX_CHANNEL_WCMD_CLOSE;
                        r_vci_ptw = 0;
                        break;
                    }
                    case RX_RELEASE :
                    {
                        printf("RX_RELEASE\n");
                        rx_channel_rcmd = RX_CHANNEL_RCMD_RELEASE;
                        r_vci_ptr = 0;
                        break;
                    }
                    case MAC_4 :
                    {
                        r_channel_mac_4[channel] = r_vci_wdata.read();
                        r_channel_mac_addr_set = r_channel_mac_addr_set.read()|(0x1<<(channel<<1));
                        break;
                    }
                    case MAC_2 :
                    {
                        r_channel_mac_2[channel] = r_vci_wdata.read();
                        r_channel_mac_addr_set = r_channel_mac_addr_set.read()|(0x1<<((channel<<1)+1));
                        break;
                    }
                    case VIS :
                    {
                        //printf("VIS before WRITE : %x\n",r_channel_active_channels.read());
                        if(r_vci_wdata.read() == 1)
                            r_channel_active_channels = r_channel_active_channels.read()|((r_vci_wdata.read()&0x1)<<channel);
                        else
                            r_channel_active_channels = r_channel_active_channels.read()&~(1<<channel);
                        //printf("VIS after WRITE : %x\n",r_channel_active_channels.get_new_value());
                        break;
                    }
                            
                }
                r_vci_fsm = VCI_IDLE;
            }
            break;
        }
        /////////////////////
        case VCI_READ_REG:   // send REG value in VCI response
        {
//printf("VCI_READ_REG\n");
  //              typename vci_param::addr_t	address = p_vci.address.read();
//printf("addr vci = %x\n",(uint32_t)address);
    //            assert ( (m_segment.contains(address)) and
      //          "ERROR in VCI_MULTI_NIC : ADDRESS is out of segment");

        //        size_t channel = r_vci_channel.read();
          //      size_t cell    = (size_t)((address & 0x00001FFF) >> 2);

            if ( p_vci.rspack.read() )
            {
                r_vci_fsm = VCI_IDLE;
            }
            break;
        }


        //////////////////////////
        /*case VCI_WRITE_RX_RELEASE:   // release tx_channel, reset r_vci_ptr,
                                    // and send VCI write response
        {
            if ( p_vci.rspack.read() )
            {
                rx_channel_rcmd = RX_CHANNEL_RCMD_RELEASE;
                r_vci_ptr       = 0;
                r_vci_fsm       = VCI_IDLE;
            }
            break;
        }
        /////////////////////
        case VCI_WRITE_MAC_4:       // set r_channel_mac_4 for selected channel,
                                    // and send VCI write response
        {
//printf("VCI_WRITE_MAC_4\n");
            if ( p_vci.rspack.read() )
            {
                size_t channel = r_vci_channel.read();
                r_channel_mac_4[channel] = r_vci_wdata.read();
//printf("address MAC 4 of channel %d = %x\n",channel,r_channel_mac_4[channel].get_new_value());
                r_vci_fsm       = VCI_IDLE;
            }
            break;
        }
        /////////////////////
        case VCI_WRITE_MAC_2:       // set r_channel_mac_2 for selected channel,
                                    // and send VCI write response
        {
            if ( p_vci.rspack.read() )
            {
                size_t channel = r_vci_channel.read();
                r_channel_mac_2[channel] = r_vci_wdata.read();
//printf("address MAC 2 of channel %d = %x\n",channel,r_channel_mac_2[channel].get_new_value());
                r_vci_fsm       = VCI_IDLE;
            }
            break;
        }*/
    } // end switch tgt_fsm

    ///////////////////////////////////////////////////////////////////////////
    // This RX_G2S module makes the GMII to STREAM format conversion.
    // It cheks the checksum, ans signals a possible error.
    // The input is the gmii_in module.
    // The output is the rx_fifo_stream, but this fifo is only used for
    // clock boundary handling, and should never be full,
    // as the consumer (RX_DES module) read all available bytes. 
    ///////////////////////////////////////////////////////////i////////////////

    // get data from PHY component
    bool    gmii_rx_dv;
    bool    gmii_rx_er;
    uint8_t gmii_rx_data;

    r_gmii_rx.get( &gmii_rx_dv, 
                   &gmii_rx_er, 
                   &gmii_rx_data, 
                   r_nic_on.read());

    // default values for fifo commands
    bool              rx_fifo_stream_write = false;
    uint16_t          rx_fifo_stream_wdata;
    
    assert( r_rx_fifo_stream.wok() and
    "ERROR in VCI_MULTI_NIC : the rs_fifo_stream should never be full");

    r_rx_g2s_dt0 = gmii_rx_data;
    r_rx_g2s_dt1 = r_rx_g2s_dt0.read();
    r_rx_g2s_dt2 = r_rx_g2s_dt1.read();
    r_rx_g2s_dt3 = r_rx_g2s_dt2.read();
    r_rx_g2s_dt4 = r_rx_g2s_dt3.read();
    r_rx_g2s_dt5 = r_rx_g2s_dt4.read();

    ///////////////////////////
    switch(r_rx_g2s_fsm.read()) 
    {
        /////////////////
        case RX_G2S_IDLE:   // waiting start of packet
        {
            if ( gmii_rx_dv and not gmii_rx_er ) // start of packet / no error
            {
                r_rx_g2s_fsm   = RX_G2S_DELAY; 
                r_rx_g2s_delay = 0;
            }
            break;
        }
        //////////////////
        case RX_G2S_DELAY:  // entering bytes in the pipe (4 cycles)
        {
            if ( not gmii_rx_dv or gmii_rx_er ) // data invalid or error
            {
                r_rx_g2s_fsm = RX_G2S_IDLE;
            }
            else if ( r_rx_g2s_delay.read() == 3 ) 
            {
                r_rx_g2s_fsm = RX_G2S_LOAD;
                r_rx_g2s_checksum = 0x00000000; // reset checksum register
            }
            else
            {
                r_rx_g2s_delay = r_rx_g2s_delay.read() + 1;
            }
            break;
        }
        /////////////////
        case RX_G2S_LOAD:   // load first in checksum accu
        {
            if ( gmii_rx_dv and not gmii_rx_er ) // data valid / no error
            {
                r_rx_g2s_checksum = (r_rx_g2s_checksum.read() >> 4) ^ crc_table[(r_rx_g2s_checksum.read() ^ (r_rx_g2s_dt4.read() >> 0)) & 0x0F];
                r_rx_g2s_checksum = (r_rx_g2s_checksum.get_new_value() >> 4) ^ crc_table[(r_rx_g2s_checksum.get_new_value() ^ (r_rx_g2s_dt4.read() >> 4)) & 0x0F];
// printf("checksum LOAD = %x\n", r_rx_g2s_checksum.get_new_value());
                r_rx_g2s_fsm      = RX_G2S_SOS;
            }
            else
            {
                r_rx_g2s_npkt_err = r_rx_g2s_npkt_err.read() + 1;
                r_rx_g2s_fsm      = RX_G2S_FAIL; // modifié => go to FAIL STATE !
            }
            break;
        }
        ////////////////
        case RX_G2S_SOS:    // write first byte in fifo_stream: SOS
        {
            if ( gmii_rx_dv and not gmii_rx_er ) // data valid / no error
            {
                r_rx_g2s_checksum = (r_rx_g2s_checksum.read() >> 4) ^ crc_table[(r_rx_g2s_checksum.read() ^ (r_rx_g2s_dt4.read() >> 0)) & 0x0F];
                r_rx_g2s_checksum = (r_rx_g2s_checksum.get_new_value() >> 4) ^ crc_table[(r_rx_g2s_checksum.get_new_value() ^ (r_rx_g2s_dt4.read() >> 4)) & 0x0F];
// printf("checksum G2S SOS = %x\n", r_rx_g2s_checksum.get_new_value());
                r_rx_g2s_fsm      = RX_G2S_LOOP;
                rx_fifo_stream_write = true;
                rx_fifo_stream_wdata = r_rx_g2s_dt5.read() | (STREAM_TYPE_SOS << 8);
            }
            else
            {
                r_rx_g2s_fsm      = RX_G2S_IDLE;
            }
            break;
        }
        /////////////////   
        case RX_G2S_LOOP:   // write one byte in fifo_stream : NEV 
        {
            rx_fifo_stream_write = true;
            rx_fifo_stream_wdata = r_rx_g2s_dt5.read() | (STREAM_TYPE_NEV << 8);
//printf("test_g2s_loop : dt4 = %x\n",r_rx_g2s_dt4.read());
            r_rx_g2s_checksum = (r_rx_g2s_checksum.read() >> 4) ^ crc_table[(r_rx_g2s_checksum.read() ^ (r_rx_g2s_dt4.read() >> 0)) & 0x0F];
            r_rx_g2s_checksum = (r_rx_g2s_checksum.get_new_value() >> 4) ^ crc_table[(r_rx_g2s_checksum.get_new_value() ^ (r_rx_g2s_dt4.read() >> 4)) & 0x0F];
//printf("checksum G2S LOOP = %x\n", r_rx_g2s_checksum.get_new_value());

            if ( not gmii_rx_dv and not gmii_rx_er ) // end of paquet
            {
                r_rx_g2s_fsm = RX_G2S_END;
            }
            else if ( gmii_rx_dv and gmii_rx_er ) // error
            {
                r_rx_g2s_fsm = RX_G2S_FAIL;
                r_rx_g2s_npkt_err = r_rx_g2s_npkt_err.read() + 1;
            }
            else if ( not gmii_rx_dv and gmii_rx_er ) // error extend
            {
                r_rx_g2s_npkt_err = r_rx_g2s_npkt_err.read() + 1;
                r_rx_g2s_fsm = RX_G2S_FAIL;
            }
            break;
        }
        ////////////////
        case RX_G2S_END:    // signal end of packet: EOS or ERR depending on checksum
        {
            uint32_t check = (uint32_t)r_rx_g2s_dt4.read()       |
                             (uint32_t)r_rx_g2s_dt3.read() << 8  | 
                             (uint32_t)r_rx_g2s_dt2.read() << 16 | 
                             (uint32_t)r_rx_g2s_dt1.read() << 24 ; 

//printf("\nCHECK = %x\n",check);
//printf("CHECKSUM_READ = %x\n",r_rx_g2s_checksum.read());
                //
                r_rx_g2s_npkt = r_rx_g2s_npkt.read() + 1;
//printf("r_rx_npkt = %d\n",r_rx_g2s_npkt.read());

            if ( r_rx_g2s_checksum.read() == check )
            {
//printf("CHECKSUM OK !\n");
                rx_fifo_stream_write = true;
                rx_fifo_stream_wdata = r_rx_g2s_dt5.read() | (STREAM_TYPE_EOS << 8);
                r_rx_g2s_npkt_crc_success = r_rx_g2s_npkt_crc_success.read() + 1;
//printf("r_rx_npkt_crc_success = %d\n",r_rx_g2s_npkt_crc_success.read());
            }
            else
            {
//printf("CHECKSUM KO !\n");
                rx_fifo_stream_write = true;
                rx_fifo_stream_wdata = r_rx_g2s_dt5.read() | (STREAM_TYPE_ERR << 8);
                r_rx_g2s_npkt_crc_fail = r_rx_g2s_npkt_crc_fail.read() + 1;
            }

            if ( gmii_rx_dv and not gmii_rx_er ) // start of packet / no error
            {
//printf("ENTERING RX_G2S_END avec RETOUR sur DELAY\n");
                r_rx_g2s_fsm   = RX_G2S_DELAY;
                r_rx_g2s_delay = 0;
            }
            else 
            {
//printf("ENTERING RX_G2S_END avec RETOUR sur IDLE\n");
                r_rx_g2s_fsm   = RX_G2S_IDLE;
            }
            break;
        }
        /////////////////
        case RX_G2S_EXTD:   // waiting end of extend to signal error
        {
            if ( not gmii_rx_er ) 
            {
                r_rx_g2s_fsm   = RX_G2S_ERR;
            }
            break;
        }
        ////////////////
        case RX_G2S_ERR:  // signal error: ERR
        {
            rx_fifo_stream_write = true;
            rx_fifo_stream_wdata = r_rx_g2s_dt5.read() | (STREAM_TYPE_ERR << 8);

            if ( gmii_rx_dv and not gmii_rx_er ) // start of packet / no error
            {
                r_rx_g2s_fsm   = RX_G2S_DELAY;
                r_rx_g2s_delay = 0;
            }
            else 
            {
                r_rx_g2s_fsm   = RX_G2S_IDLE;
            }
            break;
        }
        /////////////////
        case RX_G2S_FAIL: // waiting end of paquet to signal error
        {
            if ( not gmii_rx_dv )
            {
                r_rx_g2s_fsm   = RX_G2S_ERR;
            }
            break;
        }
    } // end switch rx_g2s_type_fsm

    ///////////////////////////////////////////////////////////////////////////
    // This RX_DES module is in charge of deserialisation (4 bytes -> 1 word).
    // - The input is the rx_fifo_stream, respecting the stream format:
    //   8 bits data + 2 bits type.
    // - The output is the rx_fifo_multi that can store a full paquet.
    // It is also charge of discarding input packets in two cases:
    // - if a checksum error is reported by the RS_G2S FSM
    // - if there not space in the rx_fifo_multi
    ///////////////////////////////////////////////////////////i////////////////

    bool              rx_fifo_stream_read    = true;
    fifo_multi_wcmd_t rx_fifo_multi_wcmd     = FIFO_MULTI_WCMD_NOP;
    uint32_t          rx_fifo_multi_wdata    = 0;
    uint32_t          rx_fifo_multi_padding  = 0;  // only used for the last word
    bool              rx_des_packet_overflow = false;
//printf("valeur du bytes counter : %d\n", r_rx_des_counter_bytes.read()); 
    switch (r_rx_des_fsm.read())
    {
        case RX_DES_READ_0 :
        {
//printf("RX_DES_READ_0\n");
                uint16_t data;
                uint32_t type;
                
                data = r_rx_fifo_stream.read();
                type = (data >> 8) & 0x3;

                r_rx_des_data[0] = (uint8_t)(data & 0xFF);

                r_rx_des_counter_bytes = 1;

                if ( r_rx_fifo_stream.rok() and (type == STREAM_TYPE_SOS) )
                {
                    r_rx_des_fsm = RX_DES_READ_1;
                }
                break;
        }

        case RX_DES_READ_1 :
        {
//printf("RX_DES_READ_1\n");
            uint16_t data;
            uint32_t type;

            data = r_rx_fifo_stream.read();
            type = (data >> 8) & 0x3;

            r_rx_des_data[1] = (uint8_t)(data & 0xFF);

            if ( r_rx_fifo_stream.rok() )
            {
                r_rx_des_counter_bytes = r_rx_des_counter_bytes.read() + 1;

                if ( type == STREAM_TYPE_NEV )
                {
                    r_rx_des_fsm = RX_DES_READ_2;
                }

                else
                {
                    r_rx_des_fsm = RX_DES_READ_0;
                }
            }
            break;
        }

        case RX_DES_READ_2:  
	    {
//printf("RX_DES_READ_2\n");
		    uint16_t data;
		    uint32_t type;
		
		    data = r_rx_fifo_stream.read();
		    type = (data >> 8) & 0x3;
		
		    r_rx_des_data[2] = (uint8_t)(data & 0xFF);
		
		    if ( r_rx_fifo_stream.rok() )
		    {

                r_rx_des_counter_bytes = r_rx_des_counter_bytes.read() + 1;

			    if (type == STREAM_TYPE_NEV)
			    {
				    r_rx_des_fsm			= RX_DES_READ_3;
			    }
			    
                else
			    {
				    r_rx_des_fsm			= RX_DES_READ_0;
			    }
		    }
		    break;
	    }

        case RX_DES_READ_3:
	    {
//printf("RX_DES_READ_3\n");
		    uint16_t data;
		    uint32_t type;
		
		    data = r_rx_fifo_stream.read();
		    type = (data >> 8) & 0x3;
		
		    r_rx_des_data[3]	= (uint8_t)(data & 0xFF);
		
		    if ( r_rx_fifo_stream.rok() )
		    {
                r_rx_des_counter_bytes = r_rx_des_counter_bytes.read() + 1;

			    if ( (type == STREAM_TYPE_NEV) and r_rx_fifo_multi.wok() )
			    {
				    r_rx_des_fsm = RX_DES_READ_WRITE_0;
			    }			
			
                else
			    {
				    r_rx_des_fsm = RX_DES_READ_0;
			    }
		    }
		    break;
	    }

        case RX_DES_READ_WRITE_0:
	    {
//printf("RX_DES_READ_WRITE_0\n");
		    uint16_t data;
		    uint32_t type;
		
		    // write previous word into fifo_multi
		    rx_fifo_multi_wcmd  = FIFO_MULTI_WCMD_WRITE;
		    rx_fifo_multi_wdata = (uint32_t)(r_rx_des_data[0].read()      ) |
			    				  (uint32_t)(r_rx_des_data[1].read() << 8 ) |
				    			  (uint32_t)(r_rx_des_data[2].read() << 16) |
					    		  (uint32_t)(r_rx_des_data[3].read() << 24) ;
		
		    // Read new word
            data                = r_rx_fifo_stream.read();
            type                = (data >> 8) & 0x3;

		    r_rx_des_data[0]	= (uint8_t)(data & 0xFF);
		    r_rx_des_padding	= 3;
		
		   
		    if ( r_rx_fifo_stream.rok() )     // do nothing if we cannot read
		    {
                 r_rx_des_counter_bytes = r_rx_des_counter_bytes.read() + 1;

    		    if ( r_rx_des_counter_bytes.get_new_value() > (1518-4) )
	    	    {
printf("PACKET OVERFLOW\n");
			        rx_des_packet_overflow = true;
		        }

			    if (type == STREAM_TYPE_NEV and !rx_des_packet_overflow)
			    {
				    r_rx_des_fsm = RX_DES_READ_WRITE_1;
			    }

			    else if ( (type == STREAM_TYPE_EOS) and r_rx_fifo_multi.wok() and !rx_des_packet_overflow )
			    {
				    r_rx_des_fsm = RX_DES_WRITE_LAST;

                    if ( r_rx_des_counter_bytes.get_new_value() < (64-4) )
                    {
printf("PACKET TOO SMALL\n");
                        r_rx_des_npkt_small = r_rx_des_npkt_small.read() + 1;
                        r_rx_des_fsm = RX_DES_WRITE_CLEAR;
                    }

			    }
			    
                else
			    {
                    if(rx_des_packet_overflow)
                        r_rx_des_npkt_overflow = r_rx_des_npkt_overflow.read() + 1;

                    if(!r_rx_fifo_multi.wok())
                        r_rx_des_npkt_err_mfifo_full = r_rx_des_npkt_err_mfifo_full.read() + 1;
				    r_rx_des_fsm = RX_DES_WRITE_CLEAR;
			    }
		    }
		    break;
	    }


       	case RX_DES_READ_WRITE_1:
    	{
//printf("RX_DES_READ_WRITE_1\n");
	    	uint16_t data;
		    uint32_t type;
		
		    data = r_rx_fifo_stream.read();
		    type = (data >> 8) & 0x3;
		
		    r_rx_des_data[1]	= (uint8_t)(data & 0xFF);
		
		    r_rx_des_padding	= 2;
		
		
		    if ( r_rx_fifo_stream.rok() )     // do nothing if we cannot read
		    {
                
		        r_rx_des_counter_bytes = r_rx_des_counter_bytes.read() + 1;		
		        
                if ( r_rx_des_counter_bytes.get_new_value() > (1518-4) )
		        {
printf("PACKET OVERFLOW\n");
			        rx_des_packet_overflow = true;
		        }

			    if (type == STREAM_TYPE_NEV and !rx_des_packet_overflow)
			    {
				    r_rx_des_fsm = RX_DES_READ_WRITE_2;
			    }

			    else if ( (type == STREAM_TYPE_EOS) and r_rx_fifo_multi.wok() and !rx_des_packet_overflow )
			    {
				    r_rx_des_fsm = RX_DES_WRITE_LAST;

                    if ( r_rx_des_counter_bytes.get_new_value() < (64-4) )
                    {
printf("PACKET TOO SMALL\n");
                        r_rx_des_npkt_small = r_rx_des_npkt_small.read() + 1;
                        r_rx_des_fsm = RX_DES_WRITE_CLEAR;
                    }
			    }

			    else
			    {
                    if(rx_des_packet_overflow)
                        r_rx_des_npkt_overflow = r_rx_des_npkt_overflow.read() + 1;
                    if(!r_rx_fifo_multi.wok())
                        r_rx_des_npkt_err_mfifo_full = r_rx_des_npkt_err_mfifo_full.read() + 1;
				    r_rx_des_fsm = RX_DES_WRITE_CLEAR;
			    }
		    }
		    break;
	    }

       	case RX_DES_READ_WRITE_2:
    	{
//printf("RX_DES_READ_WRITE_2\n");
	    	uint16_t data;
		    uint32_t type;
		
		    data = r_rx_fifo_stream.read();
		    type = (data >> 8) & 0x3;
		
		    r_rx_des_data[2]	= (uint8_t)(data & 0xFF);
		
		    r_rx_des_padding	= 1;
		
		    		
		    if ( r_rx_fifo_stream.rok() )     // do nothing if we cannot read
		    {
                r_rx_des_counter_bytes = r_rx_des_counter_bytes.read() + 1;		
    		    if ( r_rx_des_counter_bytes.get_new_value() > (1518-4) )
    		    {
printf("PACKET OVERFLOW\n");
	    	    	 rx_des_packet_overflow = true;
		        }

			    if (type == STREAM_TYPE_NEV and !rx_des_packet_overflow)
			    {
				    r_rx_des_fsm = RX_DES_READ_WRITE_3;
			    }

			    else if ( (type == STREAM_TYPE_EOS) and r_rx_fifo_multi.wok() and !rx_des_packet_overflow )
			    {
				    r_rx_des_fsm = RX_DES_WRITE_LAST;

                    if ( r_rx_des_counter_bytes.get_new_value() < (64-4) )
                    {
printf("PACKET TOO SMALL\n");
                        r_rx_des_npkt_small = r_rx_des_npkt_small.read() + 1;
                        r_rx_des_fsm = RX_DES_WRITE_CLEAR;
                    }
			    }

			    else
			    {
                    if(rx_des_packet_overflow)
                        r_rx_des_npkt_overflow = r_rx_des_npkt_overflow.read() + 1;
                    if(!r_rx_fifo_multi.wok())
                        r_rx_des_npkt_err_mfifo_full = r_rx_des_npkt_err_mfifo_full.read() + 1;
				    r_rx_des_fsm = RX_DES_WRITE_CLEAR;
			    }
		    }
		    break;
	    }
       	
        case RX_DES_READ_WRITE_3:
    	{
//printf("RX_DES_READ_WRITE_3\n");
	    	uint16_t data;
		    uint32_t type;
		
		    data = r_rx_fifo_stream.read();
		    type = (data >> 8) & 0x3;
		
		    r_rx_des_data[3]	= (uint8_t)(data & 0xFF);
		
		    r_rx_des_padding	= 0;
		
		   
	        if ( r_rx_fifo_stream.rok() )     // do nothing if we cannot read
		    { 
                r_rx_des_counter_bytes = r_rx_des_counter_bytes.read() + 1;		
		    
                if ( r_rx_des_counter_bytes.get_new_value() > (1518-4) )
		        {
printf("PACKET OVERFLOW\n");
			        rx_des_packet_overflow = true;
		        }


			    if ( (type == STREAM_TYPE_NEV) and r_rx_fifo_multi.wok() and !rx_des_packet_overflow )
			    {
				    r_rx_des_fsm = RX_DES_READ_WRITE_0;
			    }

			    else if ( (type == STREAM_TYPE_EOS) and r_rx_fifo_multi.wok() and !rx_des_packet_overflow )
			    {
				    r_rx_des_fsm = RX_DES_WRITE_LAST;

                    if (r_rx_des_counter_bytes.get_new_value() < (64-4) )
                    {
                        
printf("PACKET TOO SMALL\n");
                        r_rx_des_npkt_small = r_rx_des_npkt_small.read() + 1;
                        r_rx_des_fsm = RX_DES_WRITE_CLEAR;
                    }

			    }
			    else
			    {
                    if(rx_des_packet_overflow)
                        r_rx_des_npkt_overflow = r_rx_des_npkt_overflow.read() + 1;
                    if(!r_rx_fifo_multi.wok())
                        r_rx_des_npkt_err_mfifo_full = r_rx_des_npkt_err_mfifo_full.read() + 1;
				    r_rx_des_fsm = RX_DES_WRITE_CLEAR;
			    }
		    }
		    break;
	    }

        case RX_DES_WRITE_LAST:     // write last word in rx_fifo_multi
	    {
//printf("RX_DES_WRITE_LAST\n");
		    //uint32_t data;
            uint32_t mask = 0xFFFFFFFF; // mask used for wdata padding
		
		    rx_fifo_multi_wdata 	= (uint32_t)(r_rx_des_data[0].read()      ) |
			    			          (uint32_t)(r_rx_des_data[1].read() << 8 ) |
				    			      (uint32_t)(r_rx_des_data[2].read() << 16) |
					    		      (uint32_t)(r_rx_des_data[3].read() << 24) ;
							  
		    /*if      (rx_des_s_padding == 3) data &= 0xFF; // 3 MSbyte = 0
		    else if (rx_des_s_padding == 2) data &= 0xFFFF;	// 2 MSbyte = 0
		    else if (rx_des_s_padding == 1) data &= 0xFFFFFF; // MSbyte = 0*/
		    
            rx_fifo_multi_wdata = rx_fifo_multi_wdata & (mask>>(r_rx_des_padding.read()<<3)); // useless Bytes are set to 0
;							  
		    rx_fifo_multi_wcmd  = FIFO_MULTI_WCMD_LAST;
		    rx_fifo_multi_padding = r_rx_des_padding.read();
		    
            r_rx_des_npkt_write_mfifo_success = r_rx_des_npkt_write_mfifo_success.read() + 1;
            //r_rx_des_counter_bytes = 1;
		    rx_des_packet_overflow = false;
	        r_rx_des_fsm = RX_DES_READ_0;	
		    break;
	    }

	    case RX_DES_WRITE_CLEAR :
	    {
//printf("RX_DES_WRITE_CLEAR\n");
		    rx_fifo_multi_wcmd  = FIFO_MULTI_WCMD_CLEAR;

		   // r_rx_des_counter_bytes = 1;
		    rx_des_packet_overflow = false;
            
            r_rx_des_npkt_err_in_des = r_rx_des_npkt_err_in_des.read() + 1;
//printf("r_rx_des_npkt_err_in_des = %d\n",r_rx_des_npkt_err_in_des.read());
		
		    r_rx_des_fsm        = RX_DES_READ_0;
	    }
    } // end swich rx_des_fsm           


    /*
    // default values for fifo commands
    bool              rx_fifo_stream_read   = true;
    fifo_multi_wcmd_t rx_fifo_multi_wcmd    = FIFO_MULTI_WCMD_NOP;
    uint32_t          rx_fifo_multi_wdata   = 0;
    uint32_t          rx_fifo_multi_padding = 0;  // only used for the last word

    switch(r_rx_des_fsm.read()) 
    {
        ///////////////////////
        case RX_DES_READ_FIRST:    // read first 4 bytes in fifo_stream
        {
    //        printf("RX_DES_READ_FIRST\n");
            size_t   index = r_rx_des_byte_index.read();
            uint16_t data;
            uint32_t type;
            r_rx_des_counter_word = 0;

            if ( r_rx_fifo_stream.rok() ) // do nothing if we cannot read
            {
                data                = r_rx_fifo_stream.read();
                type                = (data >> 8) & 0x3;

                if ( ((index == 0) and (type != STREAM_TYPE_SOS)) or
                     ((index != 0) and (type != STREAM_TYPE_NEV)) )  // illegal byte type
                {
                    r_rx_des_byte_index     = 0;
                }
                else              
                {
                    r_rx_des_data[index]    = (uint8_t)(data & 0xFF);
                    if ( index == 3 )                               // last byte in word
                    {
                        r_rx_des_byte_index = 0;
                        r_rx_des_fsm        = RX_DES_READ_WRITE;
                    }
                    else
                    {
                        r_rx_des_byte_index  = index + 1;
                    }
                }
            }
            break;
        }
        ///////////////////////
        case RX_DES_READ_WRITE:     // read next 4 bytes in fifo_stream
                                    // and write one word in fifo_multi
                                    // when reading the first byte in fifo_stream
        {
  //          printf("RX_DES_READ_WRITE\n");
            size_t   index = r_rx_des_byte_index.read();
            uint16_t data;
            uint32_t type;
            
            if ( index == 0 )       // accessing both fifo_stream & fifo_multi  
            {
                if ( r_rx_fifo_stream.rok() )     // do nothing if we cannot read
                {
                    if ( r_rx_fifo_multi.wok() )  // both fifos OK => read & write
                    {
                        rx_fifo_stream_read = true;
                        data                = r_rx_fifo_stream.read();
                        type                = (data >> 8) & 0x3;

                        if ( (type == STREAM_TYPE_SOS) or 
                             (type == STREAM_TYPE_ERR) )    // illegal type => drop packet
                        {
                            //printf("ILLEGAL TYPE FOR DES_READ_WRITE (INDEX = 0)\n\n");
                            if(r_rx_des_dirty.read()) rx_fifo_multi_wcmd = FIFO_MULTI_WCMD_CLEAR;
                            rx_fifo_multi_wcmd  = FIFO_MULTI_WCMD_CLEAR;
                            r_rx_des_fsm         = RX_DES_READ_FIRST;
                            r_rx_des_byte_index  = 0;
                            r_rx_des_dirty       = false;
                        }
                        else // legal types : STREAM_TYPE_NEV or STREAM_TYPE_EOS
                        {
                            //printf("LEGAL TYPE FOR DES_READ_WRITE (INDEX = 0)\n\n");
                            // write previous word into fifo_multi
                            rx_fifo_multi_wcmd  = FIFO_MULTI_WCMD_WRITE;
                            rx_fifo_multi_wdata = (uint32_t)(r_rx_des_data[0].read()      ) |
                                                  (uint32_t)(r_rx_des_data[1].read() << 8 ) |
                                                  (uint32_t)(r_rx_des_data[2].read() << 16) |
                                                  (uint32_t)(r_rx_des_data[3].read() << 24) ;
                            r_rx_des_counter_word = r_rx_des_counter_word.read() + 1;
                            r_rx_des_dirty       = true;

                            // register read data
                            r_rx_des_data[index] = (uint8_t)(data & 0xFF);
                            if ( type == STREAM_TYPE_EOS ) 
                            {
                                r_rx_des_fsm = RX_DES_WRITE_LAST;
                            }
                            else
                            {
                                r_rx_des_byte_index  = index + 1;
                            }
                        }
                    }
                    else    // drop packet if we cannot write
                    {
                        //printf("DROP PACKET cant write\n");
                        if ( r_rx_des_dirty.read() ) rx_fifo_multi_wcmd = FIFO_MULTI_WCMD_CLEAR;
                        //rx_fifo_multi_wcmd = FIFO_MULTI_WCMD_CLEAR;
                        r_rx_des_fsm        = RX_DES_READ_FIRST;
                        r_rx_des_npkt_send_drop_mfifo_full = r_rx_des_npkt_send_drop_mfifo_full.read() + 1;
                        r_rx_des_byte_index = 0;
                        r_rx_des_dirty      = false;
                    }
                }
            }
            else  // index > 0  => accessing fifo_stream without accessing fifo_multi
            {
                if ( r_rx_fifo_stream.rok() )  // do nothing if we cannot read
                {
                    rx_fifo_stream_read = true;
                    data                = r_rx_fifo_stream.read();
                    type                = (data >> 8) & 0x3;
                    if ( (type == STREAM_TYPE_SOS) or (type == STREAM_TYPE_ERR) )
                    {
                        //printf("ILLEGAL TYPE FOR DES_READ_WRITE (INDEX > 0)\n\n");
                        rx_fifo_multi_wcmd  = FIFO_MULTI_WCMD_CLEAR;
                        r_rx_des_fsm         = RX_DES_READ_FIRST;
                        r_rx_des_byte_index  = 0;
                        r_rx_des_dirty       = false;
                    }
                    else // type == STREAM_TYPE_NEV or type == STREAM_TYPE_EOS
                    {
                        //printf("LEGAL TYPE FOR DES_READ_WRITE (INDEX > 0)\n\n");
                        r_rx_des_data[index] = (uint8_t)(data & 0xFF);
                        if ( type == STREAM_TYPE_EOS ) 
                        {
                            r_rx_des_fsm = RX_DES_WRITE_LAST;
                        }
                        else if (index == 3) 
                        {
                            r_rx_des_data[index] = (uint8_t)(data & 0xFF);
                            r_rx_des_byte_index  = 0;
                        }
                        else
                        {
                            r_rx_des_byte_index  = index + 1;
                        }
                    }
                }
            }
            break;
        }
        ///////////////////////
        case RX_DES_WRITE_LAST:     // write last word in rx_fifo_multi
                                    // or drop packet if fifo full
                                    // and read first byte in fifo_stream if possible
        {
            uint32_t mask = 0xFFFFFFFF; // mask used for wdata padding

            //printf("RX_DES_WRITE_LAST\n");
            // access fifo_multi
            if ( r_rx_fifo_multi.wok() )
            {
                rx_fifo_multi_wdata = (uint32_t)(r_rx_des_data[0].read()      ) |
                                      (uint32_t)(r_rx_des_data[1].read() << 8 ) |
                                      (uint32_t)(r_rx_des_data[2].read() << 16) |
                                      (uint32_t)(r_rx_des_data[3].read() << 24) ;
                rx_fifo_multi_wcmd  = FIFO_MULTI_WCMD_LAST;
                r_rx_des_counter_word = r_rx_des_counter_word.read() + 1;

                if (r_rx_des_counter_word.read() > 378 or r_rx_des_counter_word.read() < 15)
                {
                    rx_fifo_multi_wcmd = FIFO_MULTI_WCMD_CLEAR;
                    r_rx_des_npkt_send_drop_invplen = r_rx_des_npkt_send_drop_invplen.read() + 1;
                }
                if(rx_fifo_multi_wdata == FIFO_MULTI_WCMD_LAST)
                {
                    r_rx_des_npkt_send_write_mfifo_success = r_rx_des_npkt_send_write_mfifo_success.read() + 1;
                }
                
                rx_fifo_multi_padding = 4 - (r_rx_des_byte_index.read()+1);
                rx_fifo_multi_wdata = rx_fifo_multi_wdata & (mask>>(rx_fifo_multi_padding<<3)); // useless Bytes are set to 0

            }
            else
            {
                rx_fifo_multi_wcmd  = FIFO_MULTI_WCMD_CLEAR;
                r_rx_des_npkt_send_drop_mfifo_full = r_rx_des_npkt_send_drop_mfifo_full.read() + 1;
            }

            // access fifo_stream
            size_t   index = 0;
            uint16_t data;
            uint32_t type;

            if ( r_rx_fifo_stream.rok() )
            {
                printf (" BLABLAZLALGLGBLDGBLSLFSDLKFMSKLDFMSDFLMSDLQDF\n");
                data                = r_rx_fifo_stream.read();
                type                = (data >> 8) & 0x3;
                if ( type == STREAM_TYPE_SOS )
                {
                    r_rx_des_data[0] = (uint8_t)(data & 0xFF);
                    index = 1;
                }
            }
            r_rx_des_byte_index = index;
            r_rx_des_fsm        = RX_DES_READ_FIRST;
            r_rx_des_dirty      = false;
            break;
        }
    } // end swich rx_des_fsm
    */

    ///////////////////////////////////////////////////////////////////
    // The RX_DISPATCH FSM performs the actual transfer of
    // a packet from the rx_fifo_multi or bp_fifo_multi to 
    // the rx_channel container selected by the MAC address decoding. 
    // Packets with unexpected MAC address are discarded.
    // It imlements a round-robin priority between the two fifos.
    // Allocation is only done when a complete packet has been
    // transfered, and the DISPATCH FSM is IDLE.
    ///////////////////////////////////////////////////////////////////

    // default values for channel & fifos commands
    fifo_multi_rcmd_t    rx_fifo_multi_rcmd    = FIFO_MULTI_RCMD_NOP;
    fifo_multi_rcmd_t    bp_fifo_multi_rcmd    = FIFO_MULTI_RCMD_NOP;
    //rx_channel_wcmd_t    rx_channel_wcmd       = RX_CHANNEL_WCMD_NOP;
    rx_channel_wcmd_t    rx_channel_wcmd[m_channels];
    for(size_t j = 0; j < m_channels; j++)
        rx_channel_wcmd[j] = RX_CHANNEL_WCMD_NOP;
    
    uint32_t             rx_channel_wdata      = 0;
    uint32_t             rx_channel_padding    = 0;

    switch( r_rx_dispatch_fsm.read() )
    {
        case RX_DISPATCH_IDLE:  // ready to start a new packet transfer
        {
//printf("ENTERING RX_DISPATCH_IDLE\n");
            if ( r_rx_dispatch_bp.read() )  // previously allocated to bp_fifo
            {
                if ( r_rx_fifo_multi.rok() ) r_rx_dispatch_bp = false;
            }
            else                            // previously allocated to rx_fifo
            {
                if ( r_bp_fifo_multi.rok() ) r_rx_dispatch_bp = true;
            }
      
            if ( r_bp_fifo_multi.rok() or r_rx_fifo_multi.rok() ) // packet available
            {
                r_rx_dispatch_fsm = RX_DISPATCH_GET_PLEN;
            }
            break;
        }
        case RX_DISPATCH_GET_PLEN: // get packet length from fifo
        {   
//printf("\nRX_DISPATCH_GET_PLEN\n");
            uint32_t plen;
            
            if ( r_rx_dispatch_bp.read() ) plen = r_bp_fifo_multi.plen();  
            else                           plen = r_rx_fifo_multi.plen();  
            r_rx_dispatch_plen = plen;
            if ( (plen & 0x3) == 0 ) r_rx_dispatch_words = plen >> 2;
            else                     r_rx_dispatch_words = (plen >> 2) + 1;
            r_rx_dispatch_fsm  = RX_DISPATCH_READ_FIRST;
            break;
        }
        case RX_DISPATCH_READ_FIRST: // read first word from fifo
        {
//printf("RX_DISPATCH_READ_FIRST\n");
            if ( r_rx_dispatch_bp.read() )
            {
                bp_fifo_multi_rcmd       = FIFO_MULTI_RCMD_READ;
                r_rx_dispatch_data = r_bp_fifo_multi.data();  
            }
            else
            {
                rx_fifo_multi_rcmd       = FIFO_MULTI_RCMD_READ;
                r_rx_dispatch_data = r_rx_fifo_multi.data();
                
            }
            r_rx_dispatch_words = r_rx_dispatch_words.read() - 1;
            r_rx_dispatch_fsm   = RX_DISPATCH_CHANNEL_SELECT;

            break;
        }
        case RX_DISPATCH_CHANNEL_SELECT:  // check MAC address
        {
            // we read the second data word, without modifying the fifo state
//printf("ENTERING RX_DISPATCH_CHANNEL_SELECT\n");
            unsigned int    data_ext;
            bool            found = false;
            
            if ( r_rx_dispatch_bp.read() )  data_ext = r_bp_fifo_multi.data() & 0x0000FFFF;  
            else                            data_ext = r_rx_fifo_multi.data() & 0x0000FFFF;
            
            if ((r_rx_dispatch_data.read() == 0xFFFFFFFF)  and (data_ext == 0xFFFF))
            {
                //printf("Broadcast detected\n");
                r_rx_dispatch_broadcast = true;
                r_rx_dispatch_fsm = RX_DISPATCH_GET_WOK_BROADCAST;
            }
            else
            {    
                for ( size_t k = 0 ; (k < m_channels) and not found ; k++ )
                {
                    if(((r_channel_active_channels.read()>>k)&0x1) and ((r_channel_mac_addr_set.read()>>(k<<1))&0x3))
                    {
                        if ( (r_channel_mac_4[k].read() == r_rx_dispatch_data.read() ) and
                             (r_channel_mac_2[k].read() == data_ext) )
                        {
                            found                 = true;
                            r_rx_dispatch_channel = k;
                        }
                    }
                    else{printf("channel %d not enable or not init\n",k);}
                }
                if (found) 
                    r_rx_dispatch_fsm = RX_DISPATCH_GET_WOK;
                else
                {
//printf("MAC address 4 for channel[0] = %x and MAC 4 by packet = %x\n",r_channel_mac_4[0].read(),r_rx_dispatch_data.read());
//printf("MAC address 2 for channel[0] = %x and MAC 2 by packet = %x\n",r_channel_mac_2[0].read(),data_ext);
                    r_rx_dispatch_npkt_skip_adrmac_fail = r_rx_dispatch_npkt_skip_adrmac_fail.read() + 1;
                    printf("nb pkt addrmac fail = %d\n",r_rx_dispatch_npkt_skip_adrmac_fail.read());
                    r_rx_dispatch_fsm = RX_DISPATCH_PACKET_SKIP;
                }
            }
            break;
        }
        case RX_DISPATCH_PACKET_SKIP:	// clear an unexpected packet in source fifo
        {
//printf("PACKET SKIP !!!!!!!!\n");

            if ( r_rx_dispatch_bp.read() )  
                bp_fifo_multi_rcmd = FIFO_MULTI_RCMD_SKIP;
            else
                rx_fifo_multi_rcmd = FIFO_MULTI_RCMD_SKIP;

            r_rx_dispatch_fsm = RX_DISPATCH_IDLE;
            break;
        }
        case RX_DISPATCH_GET_WOK: // test if there is an open container in selected channel
        {
//printf("ENTERING RX_DISPATCH_GET_WOK\n");
            uint32_t channel = r_rx_dispatch_channel.read();
            bool wok         = r_rx_channel[channel]->wok();
            
            if ( wok )  
                r_rx_dispatch_fsm = RX_DISPATCH_GET_SPACE;
            else
            {
                r_rx_dispatch_npkt_wchannel_fail = r_rx_dispatch_npkt_wchannel_fail.read() + 1;
                r_rx_dispatch_fsm = RX_DISPATCH_PACKET_SKIP;
            }
            break;
        }
        case RX_DISPATCH_GET_WOK_BROADCAST: // test which containers are open
        {
//printf("ENTERING RX_DISPATCH_GET_WOK_BROADCAST\n");
            //uint32_t channel = r_rx_dispatch_channel.read();
            //bool wok         = r_rx_channel[channel]->wok();
            uint32_t sel_wok = r_rx_sel_channel_wok.read(); 
            for ( size_t k = 0 ; (k < m_channels); k++ )
            {
                if(((r_channel_active_channels.read()>>k)&0x1) and ((r_channel_mac_addr_set.read()>>(k<<1))&0x3))
                {
                    //printf("wok = %x\n",r_rx_channel[k]->wok());
                    if (r_rx_channel[k]->wok() == 1)
                    {
                        sel_wok = sel_wok | (1<<k);
                        //r_rx_sel_channel_wok  = r_rx_sel_channel_wok.read()|(1<<k);
                    }
                    else
                    {
                        sel_wok = sel_wok &~ (1<<k);
                        //r_rx_sel_channel_wok  = r_rx_sel_channel_wok.read()&~(1<<k);
                    }
                    //printf("SEL (wok1) = %x\n",r_rx_sel_channel_wok.get_new_value());
                }
                else
                {
                    //printf("SEL (wok2) = %x\n",r_rx_sel_channel_wok.get_new_value());
                    //r_rx_sel_channel_wok  = r_rx_sel_channel_wok.read()&~(1<<k);
                    sel_wok = sel_wok &~ (1<<k);
                    printf("channel %d not enable or not init\n",k);
                    //printf("SEL (wok3) = %x\n",r_rx_sel_channel_wok.get_new_value());
                }
            }
            r_rx_sel_channel_wok = sel_wok;
            printf("SEL_WOK = %x\n",r_rx_sel_channel_wok.get_new_value());
            r_rx_dispatch_fsm = RX_DISPATCH_GET_SPACE_BROADCAST;
            break;
        }
        case RX_DISPATCH_GET_SPACE_BROADCAST : // test available space in channels
        {
            //uint32_t channel = r_rx_dispatch_channel.read();
            //uint32_t space   = r_rx_channel[channel]->space();
            uint32_t plen    = r_rx_dispatch_plen.read();
            uint32_t space;
            uint32_t sel_space_timeout_ok = r_rx_sel_space_timeout_ok.read();

            // Will write only if :
            // - There is enough space
            // - There is enough time to write it
            if (r_rx_sel_channel_wok.read() == 0)
            {
                printf("PKT SKIP IN BROADCAST WOK = 0\n");
                r_rx_dispatch_fsm = RX_DISPATCH_PACKET_SKIP;
                r_rx_dispatch_npkt_wchannel_fail = r_rx_dispatch_npkt_wchannel_fail.read() + 1;
            }
            else
            {
                for ( size_t k = 0 ; (k < m_channels); k++ )
                {
                    space   = r_rx_channel[k]->space();
                    if ((plen <= space) && ((((int32_t)plen >> 2) + 1) < r_rx_channel[k]->get_r_timer()))
                    {
                        //r_rx_sel_channel_wok  = r_rx_sel_channel_wok.read()|(1<<k);
                        //r_rx_sel_space_timeout_ok = r_rx_sel_space_timeout_ok.read()|(1<<k);
                        sel_space_timeout_ok = sel_space_timeout_ok | (1<<k); 
                    }
                    else
                    {
                        //r_rx_sel_channel_wok  = r_rx_sel_channel_wok.read()&~(1<<k);
                        //r_rx_sel_space_timeout_ok = r_rx_sel_space_timeout_ok.read()&~(1<<k); 
                        sel_space_timeout_ok = sel_space_timeout_ok &~ (1<<k); 
                    }
                }
                r_rx_dispatch_fsm = RX_DISPATCH_CLOSE_CONT;
                r_rx_sel_space_timeout_ok = sel_space_timeout_ok;
                printf("SEL_SPACE = %x\n",r_rx_sel_space_timeout_ok.get_new_value());
            }
            break;
        }

        case RX_DISPATCH_GET_SPACE: // test available space in selected channel
        {
            uint32_t channel = r_rx_dispatch_channel.read();
            uint32_t space   = r_rx_channel[channel]->space();
            uint32_t plen    = r_rx_dispatch_plen.read();

            // Will write only if :
            // - There is enough space
            // - There is enough time to write it
            if ((plen <= space) && ((((int32_t)plen >> 2) + 1) < r_rx_channel[channel]->get_r_timer()))
                r_rx_dispatch_fsm = RX_DISPATCH_READ_WRITE;
            else
                r_rx_dispatch_fsm = RX_DISPATCH_CLOSE_CONT;
            break;
        }
        case RX_DISPATCH_CLOSE_CONT:  // Not enough space or time to write: close container
        {
            //uint32_t channel = r_rx_dispatch_channel.read();
            uint32_t channel;
            uint32_t sel_wok;
            uint32_t sel_space_timeout_ok;
//printf("ENTERING RX_DISPATCH_GET_CLOSE\n");
            if(r_rx_dispatch_broadcast == true)
            {
                for ( size_t k = 0 ; (k < m_channels); k++ )
                {   
                    sel_wok = (r_rx_sel_channel_wok.read()>>k)&0x1;
                    sel_space_timeout_ok = (r_rx_sel_space_timeout_ok.read()>>k)&0x1;
                    if((sel_wok == 1) and (sel_space_timeout_ok == 0))
                    {
                        printf("channel %d is close\n",k);
                        rx_channel_wcmd[k]   = RX_CHANNEL_WCMD_CLOSE;
                    }
                }
                r_rx_dispatch_fsm = RX_DISPATCH_READ_WRITE;
            }
            else
            {
                channel = r_rx_dispatch_channel.read();
                rx_channel_wcmd[channel]   = RX_CHANNEL_WCMD_CLOSE;
                r_rx_dispatch_fsm = RX_DISPATCH_GET_WOK;
            }
            break; 
        }
        case RX_DISPATCH_READ_WRITE:    // read a new word from fifo and
                                        // write previous word to channel
        {
            //uint32_t channel = r_rx_dispatch_channel.read();
//printf("ENTERING RX_DISPATCH_READ_WRITE\n");
            
            uint32_t channel;
            //uint32_t sel_wcmd_broadcast;
            uint32_t sel_wok;
            uint32_t sel_space_timeout_ok;
            if(r_rx_dispatch_broadcast == true)
            {
                for ( size_t k = 0 ; (k < m_channels); k++ )
                {
                    sel_wok = (r_rx_sel_channel_wok.read()>>k)&0x1;
                    sel_space_timeout_ok = (r_rx_sel_space_timeout_ok.read()>>k)&0x1;
                    //sel_wcmd_broadcast = (r_rx_sel_channel_wok.read()>>k)&0x1;
                    //if(sel_wcmd_broadcast == 1)
                    if((sel_wok == 1) and (sel_space_timeout_ok == 1))
                    {
                        printf("channel %d is writed\n",k);
                        rx_channel_wcmd[k] = RX_CHANNEL_WCMD_WRITE;
                    }
                    //else
                        //rx_channel_wcmd[k] = RX_CHANNEL_WCMD_NOP;
                }
            }
            else
            {
                channel = r_rx_dispatch_channel.read();
                rx_channel_wcmd[channel]     = RX_CHANNEL_WCMD_WRITE;
            }
            rx_channel_wdata    = r_rx_dispatch_data.read();
            if ( r_rx_dispatch_bp.read() )
            {
                if ( r_rx_dispatch_words.read() == 1 ) bp_fifo_multi_rcmd = FIFO_MULTI_RCMD_LAST;
                else                                   bp_fifo_multi_rcmd = FIFO_MULTI_RCMD_READ;
                r_rx_dispatch_data = r_bp_fifo_multi.data();  
            }
            else
            {
                if ( r_rx_dispatch_words.read() == 1 ) rx_fifo_multi_rcmd = FIFO_MULTI_RCMD_LAST;
                else                                   rx_fifo_multi_rcmd = FIFO_MULTI_RCMD_READ;
                r_rx_dispatch_data = r_rx_fifo_multi.data();
            }
//printf("nb words left = %d\n",r_rx_dispatch_words.read());
            r_rx_dispatch_words = r_rx_dispatch_words.read() - 1;
            if (r_rx_dispatch_words.read() == 1)
            {
                r_rx_dispatch_fsm   = RX_DISPATCH_WRITE_LAST;
                break;
            }
            break;
        }
        case RX_DISPATCH_WRITE_LAST:  // write last word to selected channel
        {
//printf("ENTERING RX_DISPATCH_WRITE_LAST\n");
            uint32_t plen = r_rx_dispatch_plen.read();
            //uint32_t channel = r_rx_dispatch_channel.read();
            uint32_t channel;
            //uint32_t sel_wcmd_broadcast;
            uint32_t sel_wok;
            uint32_t sel_space_timeout_ok;

            
            if ( (plen & 0x3) == 0 )    rx_channel_padding  = 0;
            else                        rx_channel_padding = 4 - (plen & 0x3);

            if(r_rx_dispatch_broadcast == true)
            {
                for ( size_t k = 0 ; (k < m_channels); k++ )
                {
                    sel_wok = (r_rx_sel_channel_wok.read()>>k)&0x1;
                    sel_space_timeout_ok = (r_rx_sel_space_timeout_ok.read()>>k)&0x1;
                    //sel_wcmd_broadcast = (r_rx_sel_channel_wok.read()>>k)&0x1;
                    //if(sel_wcmd_broadcast == 1)
                    if((sel_wok == 1) and (sel_space_timeout_ok == 1))
                    {
                        rx_channel_wcmd[k] = RX_CHANNEL_WCMD_LAST;
                        printf("channel %d is write LAST\n",k);
                    }
                    //else
                        //rx_channel_wcmd[k] = RX_CHANNEL_WCMD_NOP;
                }
                r_rx_dispatch_broadcast = false;
            }
            else
            {
                channel = r_rx_dispatch_channel.read();
                rx_channel_wcmd[channel]     = RX_CHANNEL_WCMD_LAST;
            }

            //rx_channel_wcmd[channel]     = RX_CHANNEL_WCMD_LAST;
            rx_channel_wdata    = r_rx_dispatch_data.read();
            r_rx_dispatch_npkt_wchannel_success = r_rx_dispatch_npkt_wchannel_success.read() + 1;
            r_rx_dispatch_fsm   = RX_DISPATCH_IDLE;
            break;
        }
    } // end switch r_rx_dispatch_fsm

    /////////////////////////////////////////////////////////////////////
    // The TX_DISPATCH FSM performs the actual transfer of
    // all packet from a tx_channel[k] container to tx_fifo or bp_fifo,
    // depending on the MAC address.
    // - the by-pass fifo is a multi-buffer fifo (typically 2 Kbytes).
    // - the tx_fifo is a simple fifo (10 bits width / 2 slots depth).
    // It implements a round-robin priority between the channels.
    // A new allocation is only done when a complete container has been
    // transmitted, and the TX_DISPATCH FSM is IDLE.
    /////////////////////////////////////////////////////////////////////

    // default values for channel & fifos commands and data
    bool                 tx_fifo_stream_write  = false;
    fifo_multi_wcmd_t    bp_fifo_multi_wcmd    = FIFO_MULTI_WCMD_NOP;
    tx_channel_rcmd_t    tx_channel_rcmd       = TX_CHANNEL_RCMD_NOP;
    uint32_t             tx_fifo_stream_wdata  = 0;
    uint32_t             bp_fifo_multi_padding = 0;
    uint32_t             bp_fifo_multi_wdata   = 0;
    
    //r_tx_dispatch_dt0 = r_tx_channel[r_tx_dispatch_channel.read()]->data();
    //r_tx_dispatch_dt1 = r_tx_dispatch_dt0.read() ;
    //r_tx_dispatch_dt2 = r_tx_dispatch_dt1.read() ;
    //printf("dt0 = %x\n",r_tx_dispatch_dt0.read());
    //printf("dt1 = %x\n",r_tx_dispatch_dt1.read());
    //printf("dt2 = %x\n",r_tx_dispatch_dt2.read());

    switch( r_tx_dispatch_fsm.read() )
    {
        //////////////////////
        case TX_DISPATCH_IDLE:  // ready to start a new packet transfer
        {
            for ( size_t x = 0 ; x < m_channels ; x++ )
            {
                size_t k = (x + 1 + r_tx_dispatch_channel.read()) % m_channels;
                if ( r_tx_channel[k]->rok() ) 
                {
//printf("TX_DISPATCH_IDLE : 1 channel not empty\n");
                    r_tx_dispatch_channel = k;
                    r_tx_dispatch_fsm     = TX_DISPATCH_GET_NPKT;
                    break;
                }
            }
            break;
        }
        //////////////////////////
        case TX_DISPATCH_GET_NPKT: // get packet number from tx_channel
        {   
            uint32_t    channel   = r_tx_dispatch_channel.read();
            r_tx_dispatch_packets = r_tx_channel[channel]->npkt();
//printf("TX_DISPATCH_GET_NPKT : %d\n",r_tx_dispatch_packets.get_new_value());
            r_tx_dispatch_fsm = TX_DISPATCH_GET_PLEN;
            break;
        }
        case TX_DISPATCH_GET_PLEN: // get packet length from tx_channel
        {   
            uint32_t channel      = r_tx_dispatch_channel.read();
            uint32_t plen         = r_tx_channel[channel]->plen();
            r_tx_dispatch_first_bytes_pckt = 1;
            r_tx_dispatch_interne = 0;
            r_tx_dispatch_bytes   = plen & 0x3;
            if ( (plen & 0x3) == 0 ) r_tx_dispatch_words = plen >> 2;
            else                     r_tx_dispatch_words = (plen >> 2) + 1;
printf("TX_DISPATCH_GET_PLEN : %d %d %d\n",plen,r_tx_dispatch_bytes.get_new_value(),r_tx_dispatch_words.get_new_value() );
            if (plen < 60 )
            {
                r_tx_dispatch_fsm = TX_DISPATCH_SKIP_PKT;
                r_tx_npkt_small = r_tx_npkt_small.read() + 1;
            }
            else if (plen > 1514)
            {
                r_tx_dispatch_fsm = TX_DISPATCH_SKIP_PKT;
                r_tx_npkt_overflow = r_tx_npkt_overflow.read() + 1;
            }
            else if(!(((r_channel_active_channels.read()>>channel)&0x1) and ((r_channel_mac_addr_set.read()>>(channel<<1))&0x3)))
            {
                printf("PKT SKIP BECAUSE channel disable or not init\n");
                r_tx_dispatch_fsm = TX_DISPATCH_SKIP_PKT;
            }
            else
            {
                r_tx_dispatch_fsm  = TX_DISPATCH_READ_FIRST;
            }
            break;
        }

        case TX_DISPATCH_SKIP_PKT :
        {
            uint32_t  packets = r_tx_dispatch_packets.read();

            tx_channel_rcmd     = TX_CHANNEL_RCMD_SKIP;
            r_tx_dispatch_packets = r_tx_dispatch_packets.read() - 1;

            if(packets == 1)
                r_tx_dispatch_fsm = TX_DISPATCH_RELEASE_CONT;
            else
                r_tx_dispatch_fsm = TX_DISPATCH_GET_PLEN;
printf("SKIP PACKET IN TX_DISPATCH !\n");
            break;
        }
       
 case TX_DISPATCH_READ_FIRST: // read first word from tx_channel
        {
            //uint32_t channel    = r_tx_dispatch_channel.read();
            tx_channel_rcmd     = TX_CHANNEL_RCMD_READ;
            //r_tx_dispatch_data  = r_tx_channel[channel]->data();
            //r_tx_dispatch_mac_4_dest  = r_tx_channel[channel]->data();
            //r_tx_dispatch_mac_4_dest  = dt0.read();
            
            r_tx_dispatch_words = r_tx_dispatch_words.read() - 1;
            
            r_tx_dispatch_fsm   = TX_DISPATCH_FIFO_SELECT;
//printf("TX_DISPATCH_READ_FISRT : data is %x\n",r_tx_dispatch_data.get_new_value());
            break;
        }
        case TX_DISPATCH_FIFO_SELECT: // check MAC address
        {
            // we read the second data word, without modifying the channel state
            uint32_t        channel  = r_tx_dispatch_channel.read();
            //uint32_t        data_tmp = r_tx_channel[channel]->data();
            //uint32_t        data_mac_2_dest = data_tmp & 0x0000FFFF;
            //uint32_t        data_mac_2_src  = (data_tmp & 0xFFFF0000)>>16;
            uint32_t        data_mac_2_dest = r_tx_channel[channel]->data() & 0x0000FFFF;
            //uint32_t        data_mac_2_src  = (r_tx_channel[channel]->data() & 0xFFFF0000)>>16;
            //unsigned int    data_ext = r_tx_channel[channel]->data() & 0x0000FFFF;  
            uint32_t            found    = r_tx_dispatch_interne.read();
            //printf("TX_DISPATCH_FIFO_SELECT : @MAC 4 = %x | @MAC 2 = %x \n",r_tx_dispatch_data.read(),data_ext );

            tx_channel_rcmd = TX_CHANNEL_RCMD_READ;
            r_tx_dispatch_words = r_tx_dispatch_words.read() - 1;
            
            //r_tx_dispatch_mac_2_dest = data_mac_2_dest;
            //r_tx_dispatch_mac_2_src = data_mac_2_src;

            /*for ( size_t k = 0 ; (k < m_channels) and not found ; k++ )
            {
                if ( (r_channel_mac_4[k].read() == r_tx_dispatch_data.read() ) and
                     (r_channel_mac_2[k].read() == data_ext) ) found = true;
            }
            if ( found )    r_tx_dispatch_fsm = TX_DISPATCH_READ_WRITE_BP;
            else            r_tx_dispatch_fsm = TX_DISPATCH_WRITE_B0_TX;
            break;*/
            for ( size_t k = 0 ; (k < m_channels) and (found == 0) ; k++ )
            {
//printf("addr mac 4 channel [%d] = %x and addr mac 4  = %x\n",k,r_channel_mac_4[k].read(),r_tx_dispatch_dt0.read());
//printf("addr mac 2 channel [%d] = %x and addr mac 2  = %x\n",k,r_channel_mac_2[k].read(),data_mac_2_dest);
                if ( (r_channel_mac_4[k].read() == r_tx_dispatch_dt0.read() ) and
                     (r_channel_mac_2[k].read() == data_mac_2_dest) ) 
                {
                    printf("ENVOIE EN INTERNE\n");
                    found = 1;
                    r_tx_dispatch_interne = found;
                }
            }
            r_tx_dispatch_fsm = TX_DISPATCH_CHECK_MAC_ADDR_SRC;
            break;
        }

        case TX_DISPATCH_CHECK_MAC_ADDR_SRC :
        {
            uint32_t channel    = r_tx_dispatch_channel.read();
            uint32_t data_mac_4_src = r_tx_channel[channel]->data();
            tx_channel_rcmd     = TX_CHANNEL_RCMD_READ;
            uint32_t tmp = ((((data_mac_4_src&0x0000FFFF)<<16)|(r_tx_dispatch_dt0.read()&0xFFFF0000)>>16)|((data_mac_4_src&0x0000FFFF)<<16));
            r_tx_dispatch_data  = r_tx_dispatch_dt1.read();
            //r_tx_dispatch_mac_4_src  = data_mac_4_src;
            r_tx_dispatch_words = r_tx_dispatch_words.read() - 1;

//printf("tx_channel data = %x\n", r_tx_channel[channel]->data());
//printf("tmp = %x\n",tmp);
//printf("dt0 = %x\n",r_tx_dispatch_dt0.read());
//printf("dt1 = %x\n",r_tx_dispatch_dt1.read());
//printf("dt2 = %x\n",r_tx_dispatch_dt2.read());
//printf("addr mac 4 channel 0 = %x and addr mac 4 src = %x\n",r_channel_mac_4[channel].read(),tmp);
//printf("addr mac 2 channel 0 = %x and addr mac 2 src = %x\n",r_channel_mac_2[channel].read(),(r_tx_channel[channel]->data()&0xFFFF0000)>>16);
            if((r_channel_mac_4[channel].read() ==  (tmp)) 
                    and (r_channel_mac_2[channel].read() == ((r_tx_channel[channel]->data()&0xFFFF0000)>>16)))
            {
                if(r_tx_dispatch_interne == 1)
                    r_tx_dispatch_fsm = TX_DISPATCH_READ_WRITE_BP;
                else
                    r_tx_dispatch_fsm = TX_DISPATCH_WRITE_B0_TX;
            } 

            else
            {
                r_tx_dispatch_addr_mac_src_fail = r_tx_dispatch_addr_mac_src_fail.read() + 1;
                printf("nb addr mac src fail = %d \n",r_tx_dispatch_addr_mac_src_fail.read());
                r_tx_dispatch_fsm = TX_DISPATCH_SKIP_PKT;
            }
            break;
        }

        case TX_DISPATCH_READ_WRITE_BP: // write previous data in bp_fifo
                                        // and read a new data from selected channel
        {
//printf("TX_DISPATCH_READ_WRITE_BP\n");
            //uint32_t  channel = r_tx_dispatch_channel.read();

            uint32_t  words   = r_tx_dispatch_words.read();  

            if ( r_bp_fifo_multi.wok() )
            {     
                bp_fifo_multi_wcmd    = FIFO_MULTI_WCMD_WRITE;
                bp_fifo_multi_wdata   = r_tx_dispatch_data.read();
                //bp_fifo_multi_wdata   = r_tx_dispatch_dt2.read();
                //r_tx_dispatch_data    = r_tx_channel[channel]->data();
                r_tx_dispatch_words   = words - 1;
                if ( words == 1 )       // read last word
                {
                    tx_channel_rcmd   = TX_CHANNEL_RCMD_LAST;
                    r_tx_dispatch_fsm = TX_DISPATCH_WRITE_LAST_BP;
                    r_tx_dispatch_data = r_tx_dispatch_dt1.read();
                }
                else
                {
                    tx_channel_rcmd   = TX_CHANNEL_RCMD_READ;
                    r_tx_dispatch_data = r_tx_dispatch_dt1.read();
                }
            }
            break;
        }
        case TX_DISPATCH_WRITE_LAST_BP:  // write last word in bp_fifo
        {
printf("TX_DISPATCH_WRITE_LAST_BP\n");
            uint32_t  packets = r_tx_dispatch_packets.read();
            uint32_t  bytes   = r_tx_dispatch_bytes.read();
            uint32_t  pipe_count = r_tx_dispatch_pipe_count.read();

            if ( r_bp_fifo_multi.wok() )
            {
                if(pipe_count == 0)
                {
                    bp_fifo_multi_wcmd    = FIFO_MULTI_WCMD_LAST;
                    bp_fifo_multi_wdata   = r_tx_dispatch_data.read();
                    //bp_fifo_multi_wdata   = r_tx_dispatch_dt2.read();
                    if ( bytes == 0 )
                        bp_fifo_multi_padding = 0;
                    else
                        bp_fifo_multi_padding = 4 - bytes;
                    r_tx_dispatch_packets = packets - 1;
                    r_tx_npkt = r_tx_npkt.read() + 1;
                    r_tx_dispatch_pipe_count = 2;
                    if ( packets == 1 )
                        r_tx_dispatch_fsm = TX_DISPATCH_RELEASE_CONT; 
                    else
                        r_tx_dispatch_fsm = TX_DISPATCH_GET_PLEN; 
                }
                else
                {
                    bp_fifo_multi_wcmd = FIFO_MULTI_WCMD_WRITE;
                    bp_fifo_multi_wdata = r_tx_dispatch_data.read();
                    if(pipe_count == 2)
                        r_tx_dispatch_data = r_tx_dispatch_dt1.read();
                    if(pipe_count == 1)
                        r_tx_dispatch_data = r_tx_dispatch_dt0.read();
                    r_tx_dispatch_pipe_count = r_tx_dispatch_pipe_count.read() - 1;
                }

            }
            break;
        }
        case TX_DISPATCH_WRITE_B0_TX: // write byte 0 in tx_fifo
        {
            uint32_t words   = r_tx_dispatch_words.read();
            uint32_t bytes   = r_tx_dispatch_bytes.read();
            uint32_t packets = r_tx_dispatch_packets.read();
            uint32_t pipe_count = r_tx_dispatch_pipe_count.read();


            if ( r_tx_fifo_stream.wok() )
            {
                tx_fifo_stream_write = true;
//                tx_fifo_stream_wdata = (r_tx_dispatch_data.read() & 0x000000FF) | (STREAM_TYPE_SOS<<8);
                if ( (r_tx_dispatch_first_bytes_pckt.read() == 1))
                {
//printf("SOS\n");
                    tx_fifo_stream_wdata = (r_tx_dispatch_data.read() & 0x000000FF) | (STREAM_TYPE_SOS<<8);
                    //tx_fifo_stream_wdata = (r_tx_dispatch_dt2.read() & 0x000000FF) | (STREAM_TYPE_SOS<<8);
printf("dt2 in B0 = %x\n",r_tx_dispatch_data.read());
                    r_tx_dispatch_first_bytes_pckt = 0;
                }
                else
                {
                    tx_fifo_stream_wdata = (r_tx_dispatch_data.read() & 0x000000FF) | (STREAM_TYPE_NEV<<8);
                    //tx_fifo_stream_wdata = (r_tx_dispatch_dt2.read() & 0x000000FF) | (STREAM_TYPE_NEV<<8);
printf("dt2 in B0 = %x\n",r_tx_dispatch_data.read());
                }
                if ( (words == 0) and (bytes == 1) and (pipe_count == 0) )   // last byte to write in tx_fifo
                {
                    tx_fifo_stream_wdata         = (r_tx_dispatch_data.read() & 0x000000FF) | (STREAM_TYPE_EOS <<8);
                    //tx_fifo_stream_wdata         = (r_tx_dispatch_dt2.read() & 0x000000FF) | (STREAM_TYPE_EOS <<8);
                    r_tx_dispatch_packets = packets - 1;
                    r_tx_npkt = r_tx_npkt.read() + 1;
                    if ( packets == 1 ) r_tx_dispatch_fsm = TX_DISPATCH_RELEASE_CONT;
                    else                r_tx_dispatch_fsm = TX_DISPATCH_GET_PLEN;
                }
                else
                {
                    r_tx_dispatch_fsm = TX_DISPATCH_WRITE_B1_TX;
                }
//printf("TX_DISPATCH_WRITE_B0_TX : wdata = %x\n",tx_fifo_stream_wdata);
            }
            break;
        }
        case TX_DISPATCH_WRITE_B1_TX: // write byte 1 in tx_fifo
        {
            uint32_t    words = r_tx_dispatch_words.read();
            uint32_t    bytes = r_tx_dispatch_bytes.read(); 
            uint32_t packets = r_tx_dispatch_packets.read();
            uint32_t pipe_count = r_tx_dispatch_pipe_count.read();

            if ( r_tx_fifo_stream.wok() )
            {
                tx_fifo_stream_write = true;
                tx_fifo_stream_wdata = ((r_tx_dispatch_data.read() >> 8) & 0x000000FF) | (STREAM_TYPE_NEV<<8);
                //tx_fifo_stream_wdata = ((r_tx_dispatch_dt2.read() >> 8) & 0x000000FF) | (STREAM_TYPE_NEV<<8);
printf("dt2 in B1= %x\n",r_tx_dispatch_data.read());
                if ( (words == 0) and (bytes == 2) and(pipe_count == 0))   // last byte to write in tx_fifo
                {
                    tx_fifo_stream_wdata         = ((r_tx_dispatch_data.read()>>8) & 0x000000FF) | (STREAM_TYPE_EOS <<8);
                    //tx_fifo_stream_wdata         = ((r_tx_dispatch_dt2.read()>>8) & 0x000000FF) | (STREAM_TYPE_EOS <<8);
                    r_tx_dispatch_packets = packets - 1;
                    r_tx_npkt = r_tx_npkt.read() + 1;
                    if ( packets == 1 ) r_tx_dispatch_fsm = TX_DISPATCH_RELEASE_CONT;
                    else                r_tx_dispatch_fsm = TX_DISPATCH_GET_PLEN;
                }
                else
                {
                    r_tx_dispatch_fsm = TX_DISPATCH_WRITE_B2_TX;
                }
//printf("TX_DISPATCH_WRITE_B1_TX : wdata = %x\n", tx_fifo_stream_wdata);
            }
            break;
        }
        case TX_DISPATCH_WRITE_B2_TX: // write byte 2 in tx_fifo
        {
            uint32_t    words = r_tx_dispatch_words.read();
            uint32_t    bytes = r_tx_dispatch_bytes.read(); 
            uint32_t packets = r_tx_dispatch_packets.read();
            uint32_t pipe_count = r_tx_dispatch_pipe_count.read();

            if ( r_tx_fifo_stream.wok() )
            {
                tx_fifo_stream_write = true;
                tx_fifo_stream_wdata = ((r_tx_dispatch_data.read() >> 16) & 0x000000FF) | (STREAM_TYPE_NEV<<8);
                //tx_fifo_stream_wdata = ((r_tx_dispatch_dt2.read() >> 16) & 0x000000FF) | (STREAM_TYPE_NEV<<8);
printf("dt2 in B2= %x\n",r_tx_dispatch_data.read());
                if ( (words == 0) and (bytes == 3) and(pipe_count==0))   // last byte to write in tx_fifo
                {
                    tx_fifo_stream_wdata         = ((r_tx_dispatch_data.read()>>16) & 0x000000FF) | (STREAM_TYPE_EOS <<8);
                    //tx_fifo_stream_wdata         = ((r_tx_dispatch_dt2.read()>>16) & 0x000000FF) | (STREAM_TYPE_EOS <<8);
                    r_tx_dispatch_packets = packets - 1;
                    r_tx_npkt = r_tx_npkt.read() + 1;
                    if ( packets == 1 ) r_tx_dispatch_fsm = TX_DISPATCH_RELEASE_CONT;
                    else                r_tx_dispatch_fsm = TX_DISPATCH_GET_PLEN;
                }
                else
                {
                    r_tx_dispatch_fsm = TX_DISPATCH_READ_WRITE_TX;
                }
//printf("TX_DISPATCH_WRITE_B2_TX : wdata = %x\n",tx_fifo_stream_wdata);
            }
            break;
        }
        case TX_DISPATCH_READ_WRITE_TX: // write byte 3 in tx_fifo
                                        // and read word from selected channel
                                        // if the current word is not the last
        {
            //uint32_t channel = r_tx_dispatch_channel.read();
            uint32_t words   = r_tx_dispatch_words.read();
            uint32_t packets = r_tx_dispatch_packets.read();
            uint32_t pipe_count = r_tx_dispatch_pipe_count.read();

            if ( r_tx_fifo_stream.wok() )
            {
                tx_fifo_stream_write = true;
                tx_fifo_stream_wdata = ((r_tx_dispatch_data.read() >> 24) & 0x000000FF) | (STREAM_TYPE_NEV<<8);
                //tx_fifo_stream_wdata = ((r_tx_dispatch_dt2.read() >> 24) & 0x000000FF) | (STREAM_TYPE_NEV<<8);
printf("dt2 in R/W= %x\n",r_tx_dispatch_data.read());
                if ( words == 0 )       // last byte to write in tx_fifo
                {
                    //tx_fifo_stream_wdata         = ((r_tx_dispatch_data.read()>>24) & 0x000000FF) | (STREAM_TYPE_EOS <<8);
                    if(pipe_count == 0)
                    {
                        tx_fifo_stream_wdata         = ((r_tx_dispatch_data.read()>>24) & 0x000000FF) | (STREAM_TYPE_EOS <<8);
                        //tx_fifo_stream_wdata         = ((r_tx_dispatch_dt2.read()>>24) & 0x000000FF) | (STREAM_TYPE_EOS <<8);
                        r_tx_dispatch_packets = packets - 1;
                        r_tx_npkt = r_tx_npkt.read() + 1;
                        r_tx_dispatch_pipe_count = 2;
                        if ( packets == 1 ) r_tx_dispatch_fsm = TX_DISPATCH_RELEASE_CONT;
                        else                r_tx_dispatch_fsm = TX_DISPATCH_GET_PLEN;
                    }
                    else
                    {
                        if(pipe_count == 2)
                            r_tx_dispatch_data = r_tx_dispatch_dt1.read();
                        else if(pipe_count == 1)
                            r_tx_dispatch_data = r_tx_dispatch_dt0.read();
                        r_tx_dispatch_pipe_count = r_tx_dispatch_pipe_count.read() - 1;
                        r_tx_dispatch_fsm = TX_DISPATCH_WRITE_B0_TX;

                    }
                }
                else if ( words == 1 )  // last word to read in tx_channel
                {
                    tx_channel_rcmd     = TX_CHANNEL_RCMD_LAST;
                    r_tx_dispatch_data  = r_tx_dispatch_dt1.read();
                    r_tx_dispatch_words = words - 1;
                    r_tx_dispatch_fsm   = TX_DISPATCH_WRITE_B0_TX;
                }
                else if ( words > 1 )   // not the last word to read in tx_channel
                {
                    tx_channel_rcmd     = TX_CHANNEL_RCMD_READ;
                    r_tx_dispatch_data  = r_tx_dispatch_dt1.read();
                    r_tx_dispatch_words = words - 1;
                    r_tx_dispatch_fsm   = TX_DISPATCH_WRITE_B0_TX;
                }
//printf("TX_DISPATCH_READ_WRITE_TX : wdata : %x\n",tx_fifo_stream_wdata);
            }
            break;
        }
        case TX_DISPATCH_RELEASE_CONT: // release the container in tx_channel
        {
//printf("TX_DISPATCH_RELEASE_CONT\n");
            tx_channel_rcmd   = TX_CHANNEL_RCMD_RELEASE;
            r_tx_dispatch_fsm = TX_DISPATCH_IDLE;
            break;
        }
    } // end switch tx_dispatch_fsm   
    if(tx_channel_rcmd == TX_CHANNEL_RCMD_READ or tx_channel_rcmd == TX_CHANNEL_RCMD_LAST)
    {
        r_tx_dispatch_dt0 = r_tx_channel[r_tx_dispatch_channel.read()]->data();
        r_tx_dispatch_dt1 = r_tx_dispatch_dt0.read() ;
        r_tx_dispatch_dt2 = r_tx_dispatch_dt1.read() ;
    }
    ////////////////////////////////////////////////////////////////////////////
    // This TX_S2G FSM performs the STREAM to GMII format conversion,
    // computes the checksum, and append this checksum to the packet.
    // The input is the tx_fifo_stream. 
    // The output is the r_gmii_tx module.
    ////////////////////////////////////////////////////////////////////////////

    // default value for fifo command
    bool    tx_fifo_stream_read = false;

    switch(r_tx_s2g_fsm.read()) 
    {
        /////////////////
        case TX_S2G_IDLE:           // read one byte from stream fifo 
        {
            
            if ( r_tx_fifo_stream.rok() )
            {
                uint32_t data = r_tx_fifo_stream.read();
                uint32_t type = (data >> 8) & 0x3;

//printf("S2G IDLE : data is %x and type is %x\n",data,type);
                assert ( (type == STREAM_TYPE_SOS) and
                "ERROR in VCI_MULTI_NIC : illegal type received in TX_S2G_IDLE");

                tx_fifo_stream_read = true;
                r_tx_s2g_fsm        = TX_S2G_WRITE_DATA;
                //r_tx_s2g_checksum   = data & 0xFF;
                r_tx_s2g_data       = data & 0xFF;
//printf("test_s2g_idle : dt4 = %x\n",data&0xFF);
                //r_tx_s2g_checksum = (r_tx_s2g_checksum.read() >> 4) ^ crc_table[(r_tx_s2g_checksum.read() ^ ((data&0xFF) >> 0)) & 0x0F];
                //r_tx_s2g_checksum = (r_tx_s2g_checksum.get_new_value() >> 4) ^ crc_table[(r_tx_s2g_checksum.get_new_value() ^ ((data&0xFF) >> 4)) & 0x0F];
                r_tx_s2g_checksum = 0x00000000;
            } 
            r_gmii_tx.put(false, false,0);
            break;
        }
        //////////////////////
        case TX_S2G_WRITE_DATA:     // write one data byte into gmii_tx
                                    // and read next byte from fifo_stream
        {
//            r_gmii_tx.put(true,false,r_tx_s2g_data.read());
            if ( r_tx_fifo_stream.rok() )
            {
                uint32_t data = r_tx_fifo_stream.read();
                uint32_t type = (data >> 8) & 0x3;
//printf("S2G_WRITE_DATA : data is %x and type is %x\n",data,type);
                assert ( (type != STREAM_TYPE_SOS) and (type != STREAM_TYPE_ERR) and
                "ERROR in VCI_MULTI_NIC : illegal type received in TX_S2G_WRITE_DATA");

                tx_fifo_stream_read = true;
                r_tx_s2g_data       = data & 0xFF;
                //r_tx_s2g_checksum   = r_tx_s2g_checksum.read() + (data & 0xFF);
                //r_tx_s2g_checksum = (r_tx_s2g_checksum.read() >> 4) ^ crc_table[(r_tx_s2g_checksum.read() ^ ((data&0xFF) >> 0)) & 0x0F];
                //r_tx_s2g_checksum = (r_tx_s2g_checksum.get_new_value() >> 4) ^ crc_table[(r_tx_s2g_checksum.get_new_value() ^ ((data&0xFF) >> 4)) & 0x0F];
                r_tx_s2g_checksum = (r_tx_s2g_checksum.read() >> 4) ^ crc_table[(r_tx_s2g_checksum.read() ^ (r_tx_s2g_data.read() >> 0)) & 0x0F];
                r_tx_s2g_checksum = (r_tx_s2g_checksum.get_new_value() >> 4) ^ crc_table[(r_tx_s2g_checksum.get_new_value() ^ (r_tx_s2g_data.read() >> 4)) & 0x0F];
//printf("checksum in s2G is : %x\n",r_tx_s2g_checksum.get_new_value());
//printf("test_s2g_idle : dt4 = %x\n",data&0xFF);


                if ( type == STREAM_TYPE_EOS ) 
                {
//printf("EOS GO WRITE_LAST !!!!\n");
                    //r_tx_s2g_fsm = TX_S2G_WRITE_CS; 
                    r_tx_s2g_fsm = TX_S2G_WRITE_LAST_DATA; 
                    //r_gmii_tx.put(true,false,r_tx_s2g_data.get_new_value());
                }

            }
            else
            {
                assert (false and  
                "ERROR in VCI_MULTI_NIC : tx_fifo should not be empty");
            } 
            r_gmii_tx.put(true,false,r_tx_s2g_data.read());
            break;
        }
        ////////////////////
        case TX_S2G_WRITE_LAST_DATA :
        {
            r_tx_s2g_fsm = TX_S2G_WRITE_CS; 
            r_gmii_tx.put(true,false,r_tx_s2g_data.read());
            r_tx_s2g_checksum = (r_tx_s2g_checksum.read() >> 4) ^ crc_table[(r_tx_s2g_checksum.read() ^ (r_tx_s2g_data.read() >> 0)) & 0x0F];
            r_tx_s2g_checksum = (r_tx_s2g_checksum.get_new_value() >> 4) ^ crc_table[(r_tx_s2g_checksum.get_new_value() ^ (r_tx_s2g_data.read() >> 4)) & 0x0F];
//printf("checksum in s2G is : %x\n",r_tx_s2g_checksum.get_new_value());
            break;
        }


        ////////////////////
        case TX_S2G_WRITE_CS:       // write one cs byte into gmii_out
        {
            uint8_t gmii_data;
            if ( r_tx_s2g_index.read() == 0 ) 
            {
                gmii_data      = r_tx_s2g_checksum.read() & 0xFF;
                r_tx_s2g_index = 1;
            }
            else if ( r_tx_s2g_index.read() == 1 ) 
            {
                gmii_data      = (r_tx_s2g_checksum.read() >> 8) & 0xFF;
                r_tx_s2g_index = 2;
            }
            else if ( r_tx_s2g_index.read() == 2 ) 
            {
                gmii_data      = (r_tx_s2g_checksum.read() >> 16) & 0xFF;
                r_tx_s2g_index = 3;
            }
            else // r_tx_s2g_index == 3 
            {
                gmii_data      = (r_tx_s2g_checksum.read() >> 24) & 0xFF;
                r_tx_s2g_index = 0;
                r_tx_s2g_fsm   = TX_S2G_IDLE;
                r_tx_s2g_checksum = 0x00000000;
//printf("CS GO S2G IDLE !\n");
            }    
            r_gmii_tx.put(true,false, gmii_data);
            break;
        }
    } // end switch tx_s2g_fsm
    
    ////////////////////////////////////////////////////////////////////////////
    // update multi_fifos
    ////////////////////////////////////////////////////////////////////////////
    r_rx_fifo_multi.update( rx_fifo_multi_wcmd, 
                            rx_fifo_multi_rcmd, 
                            rx_fifo_multi_wdata, 
                            rx_fifo_multi_padding );

    r_bp_fifo_multi.update( bp_fifo_multi_wcmd, 
                            bp_fifo_multi_rcmd, 
                            bp_fifo_multi_wdata, 
                            bp_fifo_multi_padding );


    /*printf("*********** STATUS FIFO MULTI ************\n");
    printf("valeur de data : %x\n",r_rx_fifo_multi.data());
    printf("valeur de plen : %x\n",r_rx_fifo_multi.plen());
    printf("valeur de wok : %x\n",r_rx_fifo_multi.wok());
    printf("valeur de rok : %x\n",r_rx_fifo_multi.rok());
    printf("*********** END STATUS FIFO MULTI ************\n");*/

    ////////////////////////////////////////////////////////////////////////////
    // update stream_fifos 
    ////////////////////////////////////////////////////////////////////////////
    //
    //MODIFICATION READ <=> WRITE dans arguments update()
    r_rx_fifo_stream.update( rx_fifo_stream_read,
                             rx_fifo_stream_write, 
                             rx_fifo_stream_wdata );

    r_tx_fifo_stream.update( tx_fifo_stream_read,
                             tx_fifo_stream_write, 
                             tx_fifo_stream_wdata );

    /*printf("\n\n");
    printf("*********** STATUS FIFO STREAM ************\n");
    r_tx_fifo_stream.print();
    printf("*********** END STATUS FIFO STREAM ************\n");
    printf("\n\n");*/

    ////////////////////////////////////////////////////////////////////////////
    // update rx_channels
    ////////////////////////////////////////////////////////////////////////////
    for ( size_t k = 0 ; k < m_channels ; k++ )
    {
        rx_channel_rcmd_t read;
        rx_channel_wcmd_t write;

        //if ( r_rx_dispatch_channel.read() == k )    write = rx_channel_wcmd;
        //else                                        write = RX_CHANNEL_WCMD_NOP;
        write = rx_channel_wcmd[k];


        if ( r_vci_channel.read() == k )            read  = rx_channel_rcmd;
        else                                        read  = RX_CHANNEL_RCMD_NOP;

        r_rx_channel[k]->update( write,
                                read, 
                                rx_channel_wdata, 
                                rx_channel_padding );
    }
    /*printf("*********** STATUS RX_CHANNEL[0] ************\n");
    printf("valeur de data : %x\n",r_rx_channel[0]->data());
    printf("valeur de nwords : %x\n",r_rx_channel[0]->nwords());
    printf("valeur de space : %x\n",r_rx_channel[0]->space());
    printf("valeur de wok : %x\n",r_rx_channel[0]->wok());
    printf("valeur de rok : %x\n",r_rx_channel[0]->rok());
    printf("*********** END STATUS RX_CHANNEL[0] ************\n");*/

    ////////////////////////////////////////////////////////////////////////////
    // update tx_channels
    ////////////////////////////////////////////////////////////////////////////
    for ( size_t k = 0 ; k < m_channels ; k++ )
    {
        tx_channel_rcmd_t read;
        tx_channel_wcmd_t write;

        if ( r_tx_dispatch_channel.read() == k )    read  = tx_channel_rcmd;
        else                                        read  = TX_CHANNEL_RCMD_NOP;
        if ( r_vci_channel.read() == k )            write = tx_channel_wcmd;
        else                                        write = TX_CHANNEL_WCMD_NOP;

        r_tx_channel[k]->update( write, 
                                read, 
                                tx_channel_wdata );
    }
       /* printf("\n**** STATUS STAT REGISTERS *****\n");
        printf("valeur de r_rx_g2s_npkt : %d\n",r_rx_g2s_npkt.read());
        printf("valeur de r_rx_g2s_npkt_crc_success : %d\n",r_rx_g2s_npkt_crc_success.read());
        printf("valeur de r_rx_g2s_npkt_crc_fail : %d\n",r_rx_g2s_npkt_crc_fail.read());
        printf("valeur de r_rx_g2s_npkt_err : %d\n",r_rx_g2s_npkt_err.read());
        printf("valeur de r_rx_des_npkt_err_in_des : %d\n",r_rx_des_npkt_err_in_des.read());
        printf("valeur de r_rx_des_npkt_err_mfifo_full : %d\n",r_rx_des_npkt_err_mfifo_full.read());
        printf("valeur de r_rx_des_npkt_overflow : %d\n",r_rx_des_npkt_overflow.read());
        printf("valeur de r_rx_des_npkt_small : %d\n",r_rx_des_npkt_small.read());
        printf("valeur de r_rx_des_npkt_write_mfifo_success : %d\n",r_rx_des_npkt_write_mfifo_success.read());
        printf("valeur de r_rx_des_npkt_err_mfifo_full : %d\n",r_rx_des_npkt_err_mfifo_full.read());
        printf("valeur de r_rx_dispatch_npkt_skip_adrmac_fail : %d\n",r_rx_dispatch_npkt_skip_adrmac_fail.read());
        printf("valeur de r_rx_dispatch_npkt_wchannel_success : %d\n",r_rx_dispatch_npkt_wchannel_success.read());
        printf("valeur de r_rx_dispatch_npkt_wchannel_fail : %d\n",r_rx_dispatch_npkt_wchannel_fail.read());
        printf("**** END of STATUS STAT REGISTERS *****\n");*/

} // end transition

//////////////////////
tmpl(void)::genMoore()
{
    ///////////  Interrupts ////////
    for ( size_t k = 0 ; k < m_channels ; k++ )
    {
        p_rx_irq[k] = r_rx_channel[k]->rok();
        p_tx_irq[k] = r_tx_channel[k]->wok();
    }

    ////// VCI TARGET port /////// 
    size_t channel = r_vci_channel.read();
    
    switch( r_vci_fsm.read() ) {
        case VCI_IDLE:
        case VCI_WRITE_TX_BURST:
        {
            p_vci.cmdack = true;
            p_vci.rspval = false;
            break;
        }
        case VCI_READ_RX_BURST:
        {
            p_vci.cmdack = false;
            p_vci.rspval = true;
            p_vci.rdata  = r_rx_channel[channel]->data();
            p_vci.rerror = vci_param::ERR_NORMAL;
            p_vci.rsrcid = r_vci_srcid.read();
            p_vci.rtrdid = r_vci_trdid.read();
            p_vci.rpktid = r_vci_pktid.read();
            if ( r_vci_nwords == 1 ) p_vci.reop = true;
            else                     p_vci.reop = false;
            break;
        }
        /*case VCI_WRITE_TX_CLOSE:
        case VCI_WRITE_RX_RELEASE:
        case VCI_WRITE_MAC_4:
        case VCI_WRITE_MAC_2:
        {
            p_vci.cmdack = false;
            p_vci.rspval = true;
            p_vci.rdata  = 0;
            p_vci.rerror = vci_param::ERR_NORMAL;
            p_vci.rsrcid = r_vci_srcid.read();
            p_vci.rtrdid = r_vci_trdid.read();
            p_vci.rpktid = r_vci_pktid.read();
            p_vci.reop   = true;
            break;
        }*/
        case VCI_WRITE_TX_LAST:
        case VCI_WRITE_REG:
        {
            p_vci.cmdack = false;
            p_vci.rspval = true;
            p_vci.rdata  = 0;
            p_vci.rerror = vci_param::ERR_NORMAL;
            p_vci.rsrcid = r_vci_srcid.read();
            p_vci.rtrdid = r_vci_trdid.read();
            p_vci.rpktid = r_vci_pktid.read();
            p_vci.reop   = true;
            break;
        }

        case VCI_READ_REG:
        {
            p_vci.cmdack = false;
            p_vci.rspval = true;
            //p_vci.rdata  = sel_register.read();
            p_vci.rdata  = read_register((uint32_t)p_vci.address.read());
//printf("sel_register in VCI_GENMOORE = %d\n",sel_register.read());
            p_vci.rerror = vci_param::ERR_NORMAL;
            p_vci.rsrcid = r_vci_srcid.read();
            p_vci.rtrdid = r_vci_trdid.read();
            p_vci.rpktid = r_vci_pktid.read();
            p_vci.reop   = true;
            break;
        }

    } // end switch vci_fsm

} // end genMore()

/////////////////////////
tmpl(void)::print_trace()
{
    const char* vci_state_str[] = 
    {
        "VCI_IDLE",
        //"VCI_READ_TX_WOK",
        "VCI_WRITE_TX_BURST",
        "VCI_READ_RX_BURST",
        "VCI_WRITE_TX_LAST",
        //"VCI_WRITE_TX_CLOSE",
        "VCI_WRITE_REG",
        "VCI_READ_REG",
        /*"VCI_READ_RX_ROK",
        "VCI_READ_RX_PKT",
        "VCI_READ_RX_CRC_SUCCESS",
        "VCI_READ_RX_CRC_FAIL",
        "VCI_READ_RX_ERR_MII",
        "VCI_READ_RX_MFIFO_SUCCESS",
        "VCI_READ_RX_ERR_SMALL",
        "VCI_READ_RX_ERR_OVERFLOW",
        "VCI_READ_RX_ERR_MFIFO_FULL",
        "VCI_READ_RX_ERR_IN_DES",
        "VCI_READ_RX_CHANNEL_SUCCESS",
        "VCI_READ_RX_CHANNEL_FAIL",
        "VCI_READ_RX_MAC_ADDR_FAIL",
        "VCI_READ_TX_PKT",
        "VCI_READ_TX_ERR_SMALL",
        "VCI_READ_TX_ERR_OVERFLOW",*/
        //"VCI_WRITE_RX_RELEASE",
        //"VCI_WRITE_MAC_4",
        //"VCI_WRITE_MAC_2",
    };
    const char* rx_g2s_state_str[] = 
    {
        "RX_G2S_IDLE",
        "RX_G2S_DELAY",
        "RX_G2S_LOAD",
        "RX_G2S_SOS",
        "RX_G2S_LOOP",
        "RX_G2S_END",
        "RX_G2S_EXTD",
        "RX_G2S_ERR",
        "RX_G2S_FAIL",
    };
    const char* rx_des_state_str[] = 
    {
        "RX_DES_READ_0",
        "RX_DES_READ_1",
        "RX_DES_READ_2",
        "RX_DES_READ_3",
        "RX_DES_READ_WRITE_0",
        "RX_DES_READ_WRITE_1",
        "RX_DES_READ_WRITE_2",
        "RX_DES_READ_WRITE_3",
        "RX_DES_WRITE_LAST",
        "RX_DES_WRITE_CLEAR",
    };
    const char* rx_dispatch_state_str[] =
    {
        "RX_DISPATCH_IDLE",
        "RX_DISPATCH_GET_PLEN",
        "RX_DISPATCH_READ_FIRST",
        "RX_DISPATCH_CHANNEL_SELECT",
        "RX_DISPATCH_PACKET_SKIP",
        "RX_DISPATCH_GET_WOK",
        "RX_DISPATCH_GET_WOK_BROADCAST",
        "RX_DISPATCH_GET_SPACE_BROADCAST",
        "RX_DISPATCH_CLOSE_CONT",
        "RX_DISPATCH_GET_SPACE",
        "RX_DISPATCH_READ_WRITE",
        "RX_DISPATCH_WRITE_LAST",
    };
    const char* tx_dispatch_state_str[] =
    {
        "TX_DISPATCH_IDLE",
        "TX_DISPATCH_GET_NPKT",
        "TX_DISPATCH_GET_PLEN",
        "TX_DISPATCH_SKIP_PKT", 
        "TX_DISPATCH_READ_FIRST",
        "TX_DISPATCH_FIFO_SELECT",
        "TX_DISPATCH_CHECK_ADDR_MAC_SRC",
        "TX_DISPATCH_READ_WRITE_BP",
        "TX_DISPATCH_WRITE_LAST_BP",
        "TX_DISPATCH_WRITE_B0_TX",
        "TX_DISPATCH_WRITE_B1_TX",
        "TX_DISPATCH_WRITE_B2_TX",
        "TX_DISPATCH_READ_WRITE_TX",
        "TX_DISPATCH_RELEASE_CONT",
    };
    const char* tx_s2g_state_str[] =
    {
        "TX_S2G_IDLE",
        "TX_S2G_WRITE_DATA",
        "TX_S2G_WRITE_LAST_DATA",
        "TX_S2G_WRITE_CS",
    };

    std::cout << "MULTI_NIC " << name() << " : " 
              << vci_state_str[r_vci_fsm.read()]                 << " | "
              << rx_g2s_state_str[r_rx_g2s_fsm.read()]           << " | "
              << rx_des_state_str[r_rx_des_fsm.read()]           << " | "
              << rx_dispatch_state_str[r_rx_dispatch_fsm.read()] << " | "
              << tx_dispatch_state_str[r_tx_dispatch_fsm.read()] << " | "
              << tx_s2g_state_str[r_tx_s2g_fsm.read()]           << std::endl;
} // end print_trace()

////////////////////////////////////////////////////////////////////
tmpl(/**/)::VciMultiNic( sc_core::sc_module_name 		        name,
                         const soclib::common::IntTab 		    &tgtid,
                         const soclib::common::MappingTable 	&mt,
                         const size_t 				            channels,
                         const char*                            rx_file_pathname,
                         const char*                            tx_file_pathname, 
                         const size_t 				            timeout )
	    : caba::BaseModule(name),
           
          r_nic_on("r_nic_on"),
          r_channel_active_channels("r_channel_active_channels"), 
          r_channel_mac_addr_set("r_channel_mac_addr_set"),
          r_rx_sel_channel_wok ("r_rx_sel_channel_wok"),
          r_rx_dispatch_broadcast("r_rx_dispatch_broadcast"),
          r_rx_sel_space_timeout_ok("r_rx_sel_space_timeout_ok"),
          r_vci_fsm("r_vci_fsm"),
          r_vci_srcid("r_vci_srcid"),
          r_vci_trdid("r_vci_trdid"),
          r_vci_pktid("r_vci_pktid"),
          r_vci_wdata("r_vci_wdata"),
          r_vci_ptw("r_vci_ptw"),
          r_vci_ptr("r_vci_ptr"),
          r_vci_nwords("r_vci_nwords"),

          r_rx_g2s_fsm("r_rx_g2s_fsm"),
          r_rx_g2s_checksum("r_rx_g2s_checksum"),
          r_rx_g2s_dt0("r_rx_g2s_dt0"),
          r_rx_g2s_dt1("r_rx_g2s_dt1"),
          r_rx_g2s_dt2("r_rx_g2s_dt2"),
          r_rx_g2s_dt3("r_rx_g2s_dt3"),
          r_rx_g2s_dt4("r_rx_g2s_dt4"),
          r_rx_g2s_dt5("r_rx_g2s_dt5"),
          r_rx_g2s_delay("r_rx_g2s_delay"),
          r_rx_g2s_npkt("r_rx_g2s_npkt"),
          r_rx_g2s_npkt_crc_success("r_rx_g2s_npkt_crc_success"),
          r_rx_g2s_npkt_crc_fail("r_rx_g2s_npkt_crc_fail"),
          r_rx_g2s_npkt_err("r_rx_g2s_npkt_err"),

          r_rx_des_fsm("r_rx_des_fsm"),
          r_rx_des_counter_bytes("r_rx_des_counter_bytes"),
          r_rx_des_padding("r_rx_des_padding"),
          r_rx_des_data(soclib::common::alloc_elems<sc_signal<uint8_t> >("r_rx_des_data", 4)),
          //r_rx_des_byte_index("r_rx_des_byte_index"),
          //r_rx_des_dirty("r_rx_des_dirty"),
          //r_rx_des_npkt_send_drop_mfifo_full("r_rx_des_npkt_send_drop_mfifo_full"), 
          r_rx_des_npkt_err_in_des("r_rx_des_npkt_drop_in_des"),
          r_rx_des_npkt_write_mfifo_success("r_rx_des_npkt_write_mfifo_success"),
          r_rx_des_npkt_small("r_rx_des_npkt_small"),
          r_rx_des_npkt_overflow("r_rx_des_npkt_overflow"),
          r_rx_des_npkt_err_mfifo_full("r_rx_des_npkt_err_mfifo_full"),

          r_rx_dispatch_fsm("r_rx_dispatch_fsm"),
          r_rx_dispatch_channel("r_rx_dispatch_channel"),
          r_rx_dispatch_bp("r_rx_dispatch_bp"),
          r_rx_dispatch_plen("r_rx_dispatch_plen"),
          r_rx_dispatch_data("r_rx_dispatch_data"),
          r_rx_dispatch_words("r_rx_dispatch_words"),
          r_rx_dispatch_npkt_skip_adrmac_fail("r_rx_dispatch_npkt_skip_adrmac_fail"),
          r_rx_dispatch_npkt_wchannel_success("r_rx_dispatch_npkt_wchannel_success"),
          r_rx_dispatch_npkt_wchannel_fail("r_rx_dispatch_npkt_wchannel_fail"),

          r_tx_dispatch_fsm("r_tx_dispatch_fsm"),
          r_tx_dispatch_channel("r_tx_dispatch_channel"),
          r_tx_dispatch_data("r_tx_dispatch_data"),
          r_tx_dispatch_packets("r_tx_dispatch_packets"),
          r_tx_dispatch_words("r_tx_dispatch_words"),
          r_tx_dispatch_bytes("r_tx_dispatch_bytes"),
          r_tx_dispatch_first_bytes_pckt("r_tx_dispatch_first_bytes_pckt"),
          r_tx_npkt("r_tx_npkt"),
          r_tx_npkt_overflow("r_tx_npkt_overflow"),
          r_tx_npkt_small("r_tx_npkt_small"),
          r_tx_dispatch_addr_mac_src_fail("r_tx_dispatch_addr_mac_src_fail"),
          r_tx_dispatch_dt0("r_tx_dispatch_dt0"),
          r_tx_dispatch_dt1("r_tx_dispatch_dt1"),
          r_tx_dispatch_dt2("r_tx_dispatch_dt2"),
          r_tx_dispatch_interne("r_tx_dispatch_interne"),
          r_tx_dispatch_pipe_count("r_tx_dispatch_pipe_count"),
          r_tx_s2g_fsm("r_tx_s2g_fsm"),
          r_tx_s2g_checksum("r_tx_s2g_checksum"),
          r_tx_s2g_data("r_tx_s2g_data"),
          r_tx_s2g_index("r_tx_s2g_index"),

          r_rx_fifo_stream("r_rx_fifo_stream", 2),      // 2 slots of one byte
          r_rx_fifo_multi("r_rx_fifo_multi", 32, 32),   // 1024 slots of one word
          r_tx_fifo_stream("r_tx_fifo_stream", 2),      // 2 slots of one byte
          r_bp_fifo_multi("r_bp_fifo_multi", 32, 32),   // 1024 slots of one word

          r_gmii_rx("r_gmii_rx", "in-mixte.txt", 1),  // one cycle between packets
          r_gmii_tx("r_gmii_tx", "out_tx.txt"),

          m_segment(mt.getSegment(tgtid)),
          m_channels(channels),

          p_clk("p_clk"),
          p_resetn("p_resetn"),
          p_vci("p_vci"),
          p_rx_irq(soclib::common::alloc_elems<sc_core::sc_out<bool> >("p_rx_irq", channels)),
          p_tx_irq(soclib::common::alloc_elems<sc_core::sc_out<bool> >("p_tx_irq", channels))
{
    assert( (vci_param::B == 4) and 
    "VCI_MULTI_NIC error : The VCI DATA field must be 32 bits");

    assert( (channels <= 8)  and
    "VCI_MULTI_NIC error : The number of channels cannot be larger than 8");

    r_rx_channel = new NicRxChannel*[channels];
    r_tx_channel = new NicTxChannel*[channels];

    r_channel_mac_4 = new sc_signal<uint32_t>[channels];
    r_channel_mac_2 = new sc_signal<uint32_t>[channels];

    for ( size_t k=0 ; k<channels ; k++)
    {
        r_rx_channel[k] = new NicRxChannel("r_rx_channel", timeout);
        r_tx_channel[k] = new NicTxChannel("r_tx_channel");
    }

    SC_METHOD(transition);
    dont_initialize();
    sensitive << p_clk.pos();

    SC_METHOD(genMoore);
    dont_initialize();
    sensitive << p_clk.neg();
}
}}


// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

