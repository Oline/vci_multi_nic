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
 *         Sylvain Leroy <sylvain.leroy@lip6.fr>
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
//  of variable size packets. Packet length is between 60 to 1514 bytes
//  (size without CRC).
//
//  The first 34 words of a container are the container descriptor:
//      word0:       | NB_WORDS          | NB_PACKETS        |
//      word1:       | PLEN[1]           | PLEN[0]           |
//                   | ...               | ...               |
//      word33:      | PLEN[65]          | PLEN[64]          |
//
// There is at most 62 packets in a container.
// - NB_PACKETS is the actual number of packets in the container
// - NB_WORDS is the number of useful words in the container
// - PLEN[i] is the number of bytes for packet [i].
//
// The packets are stored in the (1024-34) following words,
// and the packets are word-aligned.
//
//  In order to support various protection mechanisms, each channel takes
//  a segment of 8 Kbytes in the address space:
//  - The two first 4K bytes contain the configuration and status registers
//  - The two second 4K bytes define the current container.
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

#include <fcntl.h>
#include <errno.h>


namespace soclib { namespace caba {

#define MAC_ADDR_LEN            6
#define IFG                     12
#define HARDCODED_MAX_CHAN      8
#define MTU                     1514
#define MIN_TRAME_SIZE          60

#ifdef SOCLIB_PERF_NIC
uint32_t nic_wait_start = 400000;
uint32_t nic_end = 0;
#endif

// Declaration of crc table
static uint32_t crc_table[] =
    {
        0x4DBDF21C, 0x500AE278, 0x76D3D2D4, 0x6B64C2B0,
        0x3B61B38C, 0x26D6A3E8, 0x000F9344, 0x1DB88320,
        0xA005713C, 0xBDB26158, 0x9B6B51F4, 0x86DC4190,
        0xD6D930AC, 0xCB6E20C8, 0xEDB71064, 0xF0000000
    };


#define tmpl(t) template<typename vci_param, typename RX, typename TX> t VciMultiNic<vci_param, RX, TX>


///////////////////////////////////////////
//This function is used to return the value of status registers or stats registers
tmpl(uint32_t)::read_register(uint32_t addr)
{
    uint32_t sel_register    = 0;
    //size_t   channel         = (size_t)((addr & 0x0003C000) >> 14);
    size_t   channel         = (size_t)((addr & 0x0001C000) >> 14);     // Channel number
    size_t   cell            = (size_t)((addr & 0x00001FFF) >> 2);      // Register accessed
    bool     conf_or_status  = (addr >> 12) & 0x1;                      // conf or status page
    bool     hypervisor      = (addr & 0x00020000);                     // Are we using the hypervisor pages

#ifdef SOCLIB_NIC_DEBUG
    printf("[NIC][%s]", __func__);
#endif
    if ( p_vci.rspack.read() )
        {
            if (hypervisor == 1)
                {
#ifdef SOCLIB_NIC_DEBUG
                    printf("[HV]");
#endif
                    if (conf_or_status == 0)
                        { // CONF
#ifdef SOCLIB_NIC_DEBUG
                            printf("[CONF]");
#endif
                            switch(cell)
                                {
                                case GENERAL_CHAN_MAC_ADDR_SET :
                                    {
                                        sel_register = r_channel_mac_addr_set.read();
#ifdef SOCLIB_NIC_DEBUG
                                        printf(" GENERAL_CHAN_MAC_ADDR_SET = %x\n", sel_register);
#endif
                                        break;
                                    }
                                case VIS :
                                    {
                                        sel_register = r_channel_active_channels.read();
#ifdef SOCLIB_NIC_DEBUG
                                        printf(" VIS = %x\n", sel_register);
#endif
                                        break;
                                    }
                                case GENERAL_MAC_4 : // Correspond to the chan0 mac addr
                                    {
                                        sel_register = r_channel_mac_4[0].read();
#ifdef SOCLIB_NIC_DEBUG
                                        printf(" GENERAL_MAC_4 = %x\n", sel_register);
#endif
                                        break;
                                    }
                                case GENERAL_MAC_2 : // Correspond to the chan0 mac addr
                                    {
                                        sel_register = r_channel_mac_2[0].read();
#ifdef SOCLIB_NIC_DEBUG
                                        printf(" GENERAL_MAC_2 = %x\n", sel_register);
#endif
                                        break;
                                    }
                                case NIC_ON :
                                    {
                                        sel_register = r_nic_on.read();
#ifdef SOCLIB_NIC_DEBUG
                                        printf(" NIC_ON = %x\n", sel_register);
#endif
                                        break;
                                    }
                                case BROADCAST_ENABLE :
                                    {
                                        sel_register = r_broadcast_enable.read();
#ifdef SOCLIB_NIC_DEBUG
                                        printf(" BROADCAST_ENABLE = %x\n", sel_register);
#endif
                                        break;
                                    }
                                case TDM_ENABLE :
                                    {
                                        sel_register = r_tx_tdm_enable.read();
#ifdef SOCLIB_NIC_DEBUG
                                        printf(" TDM_ENABLE = %x\n", sel_register);
#endif
                                        break;
                                    }
                                case TDM_TIMERS :
                                    {
                                        sel_register = r_tx_tdm_timer.read();
#ifdef SOCLIB_NIC_DEBUG
                                        printf(" TDM_TIMERS (Write ONLY register) (current value of the running timer, not the channel TDM timer) = %x\n", sel_register);
#endif
                                        break;
                                    }
                                default :
                                    {
#ifdef SOCLIB_NIC_DEBUG
                                        printf("[ERR] NO READ SHOULD BE ALLOWED HERE!!!\n");
#endif
                                    }
                                }
                        }
                    else // STATUS
                        {
#ifdef SOCLIB_NIC_DEBUG
                            printf("[STATUS]");
#endif
                            // Check we are in the chan to chan matrix
                            if ( (cell&0x3FF) < (TX_REG + ((m_channels-1)<<2) ) and ( (cell&0x3FF) >= TX_REG) )
                                {
                                    // chan to chan matrix
                                    uint32_t myregister = (cell&0x3FF) - TX_REG;
                                    uint32_t line = myregister / m_channels;
                                    uint32_t col = myregister % m_channels;
                                    sel_register = r_tx_channel_to_channel[line][col].read();
#ifdef SOCLIB_NIC_DEBUG
                                    printf(" Internal switching from chan %d to chan %d has value : 0x%x\n", line, col, sel_register);
#endif
                                }
                            else
                                {
                                    // regular register
                                    switch(cell&0x3FF)
                                        {
                                        case RX_PKT :
                                            {
#ifdef SOCLIB_NIC_DEBUG
                                                printf(" RX_PKT\n");
#endif
                                                sel_register = r_rx_g2s_npkt.read();
                                                break;
                                            }
                                        case RX_CRC_SUCCESS :
                                            {
#ifdef SOCLIB_NIC_DEBUG
                                                printf(" CRC_SUCCESS\n");
#endif
                                                sel_register = r_rx_g2s_npkt_crc_success.read();
                                                break;
                                            }
                                        case RX_CRC_FAIL :
                                            {
#ifdef SOCLIB_NIC_DEBUG
                                                printf(" RX_CRC_FAIL\n");
#endif
                                                sel_register = r_rx_g2s_npkt_crc_fail.read();
                                                break;
                                            }
                                        case RX_ERR_MII :
                                            {
#ifdef SOCLIB_NIC_DEBUG
                                                printf(" RX_ERR_MII\n");
#endif
                                                sel_register = r_rx_g2s_npkt_err.read();
                                                break;
                                            }
                                        case RX_MFIFO_SUCCESS :
                                            {
#ifdef SOCLIB_NIC_DEBUG
                                                printf(" RX_MFIFO_SUCCESS\n");
#endif
                                                sel_register = r_rx_des_npkt_write_mfifo_success.read();
                                                break;
                                            }
                                        case RX_ERR_SMALL :
                                            {
#ifdef SOCLIB_NIC_DEBUG
                                                printf(" RX_ERR_SMALL\n");
#endif
                                                sel_register = r_rx_des_npkt_small.read();
                                                break;
                                            }
                                        case RX_ERR_OVERFLOW :
                                            {
#ifdef SOCLIB_NIC_DEBUG
                                                printf(" RX_ERR_OVERFLOW\n");
#endif
                                                sel_register = r_rx_des_npkt_overflow.read();
                                                break;
                                            }
                                        case RX_ERR_MFIFO_FULL :
                                            {
#ifdef SOCLIB_NIC_DEBUG
                                                printf(" RX_ERR_MFIFO_FULL\n");
#endif
                                                sel_register =r_rx_des_npkt_err_mfifo_full.read();
                                                break;
                                            }
                                        case RX_ERR_IN_DES :
                                            {
#ifdef SOCLIB_NIC_DEBUG
                                                printf(" RX_ERR_IN_DES\n");
#endif
                                                sel_register = r_rx_des_npkt_err_in_des.read();
                                                break;
                                            }
                                        case RX_CHANNEL_SUCCESS :
                                            {
#ifdef SOCLIB_NIC_DEBUG
                                                printf(" RX_CHANNEL_SUCCESS\n");
#endif
                                                sel_register = r_rx_dispatch_npkt_wchannel_success.read();
                                                break;
                                            }
                                        case RX_CHANNEL_FAIL :
                                            {
#ifdef SOCLIB_NIC_DEBUG
                                                printf(" RX_CHANNEL_FAIL\n");
#endif
                                                sel_register = r_rx_dispatch_npkt_wchannel_fail.read();
                                                break;
                                            }
                                        case RX_MAC_ADDR_FAIL :
                                            {
#ifdef SOCLIB_NIC_DEBUG
                                                printf(" RX_MAC_ADDR_FAIL\n");
#endif
                                                sel_register = r_rx_dispatch_npkt_skip_adrmac_fail.read();
                                                break;
                                            }
                                        case TX_PKT :
                                            {
#ifdef SOCLIB_NIC_DEBUG
                                                printf(" TX_PKT\n");
#endif
                                                sel_register = r_tx_npkt.read();
                                                break;
                                            }
                                        case TX_ERR_SMALL :
                                            {
#ifdef SOCLIB_NIC_DEBUG
                                                printf(" ERR_SMALL\n");
#endif
                                                sel_register = r_tx_npkt_small.read();
                                                break;
                                            }
                                        case TX_ERR_OVERFLOW :
                                            {
#ifdef SOCLIB_NIC_DEBUG
                                                printf(" RX_ERR_OVERFLOW\n");
#endif
                                                sel_register = r_tx_npkt_overflow.read();
                                                break;
                                            }
                                        default :
                                            {
#ifdef SOCLIB_NIC_DEBUG
                                                printf("[ERR] NO READ SHOULD BE ALLOWED HERE!!!\n");
#endif
                                            }
                                        } // end SWITCH CASE
                                } // end if else (????)
                        } // end if else (conf or status)
                } // end HV
            else
                {
#ifdef SOCLIB_NIC_DEBUG
                    printf("[CHAN]");
#endif
                    if (conf_or_status == 0)
                        {// CONF
#ifdef SOCLIB_NIC_DEBUG
                            printf("[CONF]");
#endif
                            switch(cell)
                                {
                                case TIMEOUT :
                                    {
#ifdef SOCLIB_NIC_DEBUG
                                        printf(" TIMEOUT\n");
#endif
                                        sel_register = r_rx_channel[channel]->get_m_timeout();
                                        break;
                                    }
                                default :
                                    {
#ifdef SOCLIB_NIC_DEBUG
                                        printf("[ERR] NO READ SHOULD BE ALLOWED HERE!!!\n");
#endif
                                    }
                                }
                        }
                    else
                        {
#ifdef SOCLIB_NIC_DEBUG
                            printf("[STATUS]");
#endif
                            switch(cell&0x3FF)
                                {
                                case MAC_4 :
                                    {
#ifdef SOCLIB_NIC_DEBUG
                                        printf(" MAC4\n");
#endif
                                        sel_register = r_channel_mac_4[channel].read();
                                        break;
                                    }
                                case MAC_2 :
                                    {
#ifdef SOCLIB_NIC_DEBUG
                                        printf(" MAC2\n");
#endif
                                        sel_register = r_channel_mac_2[channel].read();
                                        break;
                                    }
                                case RX_ROK :
                                    {
#ifdef SOCLIB_NIC_DEBUG
                                        printf(" RX_ROK\n");
#endif
                                        sel_register = r_rx_channel[channel]->rok();
                                        break;
                                    }
                                case TX_WOK :
                                    {
#ifdef SOCLIB_NIC_DEBUG
                                        printf(" TX_WOK\n");
#endif
                                        sel_register = r_tx_channel[channel]->wok();
                                        break;
                                    }
                                case RX_NWORDS :
                                    {
#ifdef SOCLIB_NIC_DEBUG
                                        printf(" RX_NWORDS\n");
#endif
                                        sel_register = r_rx_channel[channel]->nwords();
                                        break;
                                    }
                                default :
                                    {
#ifdef SOCLIB_NIC_DEBUG
                                        printf("[ERR] NO READ SHOULD BE ALLOWED HERE!!!\n");
#endif
                                    }
                                }//end switch cell
                        }
                }
        }
    return sel_register;
}

/////////////////////////
tmpl(void)::transition()
{
    if (!p_resetn)
        {
#ifdef SOCLIB_NIC_DEBUG
            printf("[NIC][%s] Resetting the controller\n", __func__);
#endif
            r_vci_fsm          = VCI_IDLE;
            r_vci_ptr          = 0;
            r_vci_ptw          = 0;

#ifdef SOCLIB_PERF_NIC
            r_total_len_gmii = 0;
            r_total_len_rx_chan = 0;
            r_total_len_tx_chan = 0;
            r_total_len_tx_gmii = 0;
#endif
            r_rx_dispatch_bp = false;
            r_nic_on = 0;
            r_broadcast_enable = 0;
            r_channel_active_channels = 0;
            r_channel_mac_addr_set = 0;
            r_rx_sel_channel_wok = 0;
            r_rx_dispatch_broadcast = 0;
            r_rx_sel_space_timeout_ok = 0;

            r_rx_g2s_fsm       = RX_G2S_IDLE;
            r_rx_des_fsm       = RX_DES_READ_0;
            r_rx_dispatch_fsm  = RX_DISPATCH_IDLE;
            r_tx_dispatch_fsm  = TX_DISPATCH_IDLE;
            r_tx_s2g_fsm       = TX_S2G_IDLE;

            r_rx_g2s_dt0 = 0;
            r_rx_g2s_dt1 = 0;
            r_rx_g2s_dt2 = 0;
            r_rx_g2s_dt3 = 0;
            r_rx_g2s_dt4 = 0;
            r_rx_g2s_dt5 = 0;
            r_rx_g2s_npkt = 0;
            r_rx_g2s_npkt_crc_success = 0;
            r_rx_g2s_npkt_crc_fail = 0;
            r_rx_g2s_npkt_err = 0;

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
            r_rx_dispatch_dt0 = 0;

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
            r_tx_dispatch_broadcast = 0;
            r_tx_dispatch_channel_interne_send = 0;
            r_tx_dispatch_ifg = IFG;

            r_tx_s2g_data = 0;

            r_tx_tdm_enable = 0;
            r_tx_tdm_timer  = 0;
            r_tx_chan_sel_tdm = 0;

            for ( size_t k = 0 ; k < 4 ; k++)
                r_rx_des_data[k] = 0;

            for ( size_t k = 0 ; k < m_channels ; k++ )
                {
                    r_tx_chan_tdm_timer[k]   = 0;
                    r_tx_dispatch_packets[k] = 0;
                }

            r_rx_fifo_stream.init();
            r_rx_fifo_multi.reset();
            r_tx_fifo_stream.init();
            r_bp_fifo_multi.reset();

            r_backend_rx.reset();
            r_backend_tx.reset();

            for (size_t i = 0 ; i < m_channels ; i ++)
                {
                    for (size_t j = 0; j < m_channels ; j ++)
                        {
                            r_tx_channel_to_channel[i][j] = 0;
                        }
                }

            for ( size_t k = 0 ; k < m_channels ; k++ )
                {
                    r_rx_channel[k]->reset();
                    r_tx_channel[k]->reset();
                }

            // initialise MAC addr
            for (size_t i = 0; i < m_channels; i++)
                {
                    r_channel_mac_4[i] = (m_macaddr[0] << 24) + (m_macaddr[1] << 16) + (m_macaddr[2] << 8) + (m_macaddr[3]);
                    r_channel_mac_2[i] = (m_macaddr[4] << 8) + (m_macaddr[5] + i); // MIGHT OVERFLOW !!!!!!!!!!!!!!
                }

#ifdef SOCLIB_NIC_DEBUG
            printf("[NIC][%s] Reset done\n", __func__);
#endif
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
                        typename vci_param::addr_t	address = p_vci.address.read();
                        typename vci_param::cmd_t	cmd     = p_vci.cmd.read();
                        typename vci_param::plen_t  plen    = p_vci.plen.read();

                        assert ( ((plen & 0x3) == 0) and
                                 "ERROR in VCI_MULTI_NIC : PLEN field must be multiple of 4 bytes");

                        assert ( (m_segment.contains(address)) and
                                 "ERROR in VCI_MULTI_NIC : ADDRESS is out of segment");

                        r_vci_srcid	   = p_vci.srcid.read();
                        r_vci_trdid	   = p_vci.trdid.read();
                        r_vci_pktid	   = p_vci.pktid.read();
                        r_vci_wdata    = p_vci.wdata.read();

                        size_t channel = (size_t)((address & 0x0001C000) >> 14);
                        bool   hypervisor = (address & 0x00020000);
                        bool   burst   = (address & 0x00002000);

                        r_vci_channel  = channel;
                        r_vci_address  = (uint32_t)address;
#ifdef SOCLIB_NIC_DEBUG
                        printf("[NIC][VCI_FSM] VCI_IDLE | vci address = %x, plen = %d, channel %x, hypervisor %x, burst %x\n",(uint32_t)address, (uint16_t)plen, channel, hypervisor, burst);
#endif

                        if (hypervisor == 0)
                            {
                                assert( (channel < m_channels) and
                                        "VCI_MULTI_NIC error : The channel index (ADDR[16:14]) is too large");
                            }
                        /*if (channel > m_channels)
                          r_vci_fsm = VCI_ERROR;
                          else*/ if ( burst and (cmd == vci_param::CMD_WRITE) )      // TX_BURST transfer
                            {
                                if ( p_vci.eop.read() )
                                    r_vci_fsm = VCI_WRITE_TX_LAST;
                                else
                                    r_vci_fsm = VCI_WRITE_TX_BURST;
                            }
                        else if ( burst  and (cmd == vci_param::CMD_READ) ) // RX_BURST transfer
                            {
                                r_vci_nwords  = (size_t)(plen >> 2);
                                r_vci_fsm     = VCI_READ_RX_BURST;
                            }

                        else if ( not burst and (cmd == vci_param::CMD_READ) )           // REG read access
                            {
                                assert( p_vci.eop.read() and
                                        "ERROR in VCI_MULTI_NIC : RX_REG read access must be one flit");
                                r_vci_fsm = VCI_READ_REG;
                                /*if(p_vci.eop.read() == 0)
                                  {
                                  printf("ERROR in VCI_MULTI_NIC : RX_REG read access must be one flit\n");
                                  r_vci_fsm = VCI_ERROR;
                                  }*/
                            }

                        else if ( not burst and (cmd == vci_param::CMD_WRITE))      // REG write access
                            {
                                assert ( p_vci.eop.read() and
                                         "ERROR in VCI_MULTI_NIC : WRITE_REG write access must be one flit");
                                r_vci_fsm = VCI_WRITE_REG;
                                /*if(p_vci.eop.read() == 0)
                                  {
                                  printf("ERROR in VCI_MULTI_NIC : WRITE_REG read access must be one flit\n");
                                  r_vci_fsm = VCI_ERROR;
                                  }*/
                            }
                        else
                            {
                                assert( false and
                                        "ERROR in VCI_MULTI_NIC : illegal VCI command");
                                /*r_vci_fsm = VCI_ERROR;
                                  printf("ERROR in VCI_MULTI_NIC : illegal VCI command\n");*/
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

#ifdef SOCLIB_NIC_DEBUG
                printf("[NIC][VCI_FSM] VCI_WRITE_TX_BURST\n");
#endif
                /*if( r_tx_channel[channel]->wok() == 0)
                  {
                  r_vci_fsm = VCI_ERROR;
                  printf("ERROR in VCI_MULTI_NIC : tx_channel should not be full in VCI_WRITE_TX_BURST\n");
                  }
                  else*/
                if ( p_vci.cmdval.read() )
                    {
                        // data[i-1]
                        tx_channel_wcmd  = TX_CHANNEL_WCMD_WRITE;
                        tx_channel_wdata = r_vci_wdata.read();

                        // data[i]
                        uint32_t address = r_vci_address.read();
                        r_vci_address = p_vci.address.read();

                        assert( (((address & 0x00000FFF) >> 2) == r_vci_ptw.read()) and
                                "ERROR in VCI_MULTI_NIC : address must be contiguous in VCI_WRITE_TX_BURST");
                        /*if(((address & 0x00000FFF) >> 2) != r_vci_ptw.read())
                          {
                          r_vci_fsm = VCI_ERROR;
                          printf("ERROR in VCI_MULTI_NIC : address must be contiguous in VCI_WRITE_TX_BURST\n");
                          }
                          else
                          {*/
                        r_vci_wdata      = p_vci.wdata.read();
                        r_vci_ptw        = r_vci_ptw.read() + 1;
                        if ( p_vci.eop.read() )
                            {
                                r_vci_fsm = VCI_WRITE_TX_LAST;
                            }
                        //}
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
                        r_vci_ptw        = r_vci_ptw.read() + 1;
                        r_vci_fsm = VCI_IDLE;
#ifdef SOCLIB_NIC_DEBUG
                        printf("[NIC][VCI_FSM] VCI_WRITE_TX_LAST : write LAST in VCI TGT FSM\n");
#endif
                    }
                break;
            }
            ///////////////////////
        case VCI_READ_RX_BURST: // check pointer and send data in VCI response
            {
                size_t channel = r_vci_channel.read();
                assert ( r_rx_channel[channel]->rok() and
                         "ERROR in VCI_MULTI_NIC : rx_channel should not be empty in VCI_READ_RX_BURST");

#ifdef SOCLIB_NIC_DEBUG
                printf("[NIC][VCI_FSM] VCI_READ_RX_BURST\n");
#endif
                /*if( r_rx_channel[channel]->rok() == 0)
                  {
                  r_vci_fsm = VCI_ERROR;
                  printf("ERROR in VCI_MULTI_NIC : rx_channel should not be empty in VCI_WRITE_TX_BURST\n");
                  }
                  else*/
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
                size_t   channel                    = r_vci_channel.read();                     // Channel number
                size_t   cell                       = (size_t)((address & 0x00001FFF) >> 2);    // Register number
                bool     conf_or_status             = (address >> 12) & 0x1;                    // Configuration or Status registers' page | conf = 0 and status = 1
                bool     hypervisor                 = (address & 0x00020000);                   // Hypervisor pages or not

#ifdef SOCLIB_NIC_DEBUG
                printf("[NIC][VCI_FSM] VCI_WRITE_REG | channel %x, register %x, ", channel, cell);
                if (conf_or_status == 0)
                    printf("configuration, ");
                else
                    printf("status , ");
                printf("hypervisor %x\n", hypervisor);
#endif
                if ( p_vci.rspack.read() )
                    {
                        if (hypervisor == true) // We are in hypervisor registers pages
                            {
                                if (conf_or_status == false)
                                    {
                                        if ( cell < (TDM_TIMERS + HARDCODED_MAX_CHAN) and ( cell >= TDM_TIMERS ) ) // Are we in the registers SET_TDM_TIMER array
                                            {
                                                uint32_t chan = cell - TDM_TIMERS; // We get the channel number
                                                if (chan < m_channels)
                                                    {
#ifdef SOCLIB_NIC_DEBUG
                                                        printf("[NIC][VCI_FSM] VCI_WRITE_REG | r_tx_chan_tdm_timer[%x] change value 0x%x -> 0x%x\n",
                                                               chan, (uint32_t)r_tx_chan_tdm_timer[chan], (uint32_t)r_vci_wdata.read());
#endif
                                                        // We set the TIMER for that very channel
                                                        // We MUST do that write for every channel !!!
                                                        r_tx_chan_tdm_timer[chan] = r_vci_wdata.read();
                                                    }
                                                else
                                                    printf("[NIC][VCI_FSM] VCI_WRITE_REG | Out of TDM_TIMER array\n");
                                            }
                                        else
                                            {
                                                switch(cell)
                                                    {
                                                    case TDM_ENABLE :
                                                        {
#ifdef SOCLIB_NIC_DEBUG
                                                            printf("[NIC][VCI_FSM] VCI_WRITE_REG | r_tx_tdm_enable change value %x -> %x\n", (uint32_t)r_tx_tdm_enable, (uint32_t)r_vci_wdata.read());
#endif
                                                            r_tx_tdm_enable = r_vci_wdata.read();
                                                            break;
                                                        }
                                                    case BROADCAST_ENABLE :
                                                        {
#ifdef SOCLIB_NIC_DEBUG
                                                            printf("[NIC][VCI_FSM] VCI_WRITE_REG | r_broadcast_enable change value %x -> %x\n", (uint32_t)r_broadcast_enable, (uint32_t)r_vci_wdata.read());
#endif
                                                            r_broadcast_enable = r_vci_wdata.read();
                                                            break;
                                                        }
                                                    case NIC_ON :
                                                        {
#ifdef SOCLIB_NIC_DEBUG
                                                            printf("[NIC][VCI_FSM] VCI_WRITE_REG | r_nic_on change value %x -> %x\n", (uint32_t)r_nic_on, (uint32_t)r_vci_wdata.read());
#endif
                                                            r_nic_on = r_vci_wdata.read();
                                                            break;
                                                        }
                                                    case VIS :
                                                        {
#ifdef SOCLIB_NIC_DEBUG
                                                            printf("[NIC][VCI_FSM] VCI_WRITE_REG | VIS change value %x -> %x\n", (uint32_t)r_channel_active_channels, (uint32_t)r_vci_wdata.read());
#endif
                                                            r_channel_active_channels = r_vci_wdata.read();
                                                            break;
                                                        }
                                                    case GENERAL_CHAN_MAC_ADDR_SET :
                                                        {
#ifdef SOCLIB_NIC_DEBUG
                                                            printf("[NIC][VCI_FSM] VCI_WRITE_REG | r_channel_mac_addr_set change value %x -> %x\n", (uint32_t)r_channel_mac_addr_set, (uint32_t)r_vci_wdata.read());
#endif
                                                            r_channel_mac_addr_set = r_vci_wdata.read();
                                                            break;
                                                        }
                                                    }
                                            }
                                    }
                            }
                        else // We are in users registers pages
                            {
                                if (conf_or_status == 0) // Configuration registers
                                    {
                                        switch(cell)
                                            {
                                            case TX_CLOSE :
                                                {
#ifdef SOCLIB_NIC_DEBUG
                                                    printf("[NIC][VCI_FSM] VCI_WRITE_REG | closing TX container\n");
#endif
                                                    tx_channel_wcmd = TX_CHANNEL_WCMD_CLOSE;
                                                    r_vci_ptw = 0;
                                                    break;
                                                }
                                            case RX_RELEASE :
                                                {
#ifdef SOCLIB_NIC_DEBUG
                                                    printf("[NIC][VCI_FSM] VCI_WRITE_REG | release RX container\n");
#endif
                                                    rx_channel_rcmd = RX_CHANNEL_RCMD_RELEASE;
                                                    r_vci_ptr = 0;
                                                    break;
                                                }

                                            case TIMEOUT :
                                                {
                                                    if ( r_vci_wdata.read() < 379 )
                                                        {
#ifdef SOCLIB_NIC_DEBUG
                                                            printf("[NIC][VCI_FSM] VCI_WRITE_REG | TIMEOUT too SMALL\n");
#endif
                                                        }
                                                    else
                                                        {
#ifdef SOCLIB_NIC_DEBUG
                                                            printf("[NIC][VCI_FSM] VCI_WRITE_REG | set timeout = %x\n", (uint32_t)r_vci_wdata.read());
#endif
                                                            r_rx_channel[channel]->set_timeout(r_vci_wdata.read());
                                                        }
                                                    break;
                                                }
                                            }
                                    }
                                else // Status registers
                                    {
                                        switch (cell&0x3FF)
                                            {
                                            case MAC_4 :
                                                {
#ifdef SOCLIB_NIC_DEBUG
                                                    printf("[NIC][VCI_FSM] VCI_WRITE_REG | Writing MAC4 addr : %x\n", (uint32_t)r_vci_wdata.read());
#endif
                                                    r_channel_mac_4[channel] = r_vci_wdata.read();
                                                    break;
                                                }
                                            case MAC_2 :
                                                {
#ifdef SOCLIB_NIC_DEBUG
                                                    printf("[NIC][VCI_FSM] VCI_WRITE_REG | Writing MAC2 addr : %x\n", (uint32_t)r_vci_wdata.read());
#endif
                                                    r_channel_mac_2[channel] = r_vci_wdata.read();
                                                    break;
                                                }
                                            }
                                    }
                            }
                    }
                r_vci_fsm = VCI_IDLE;
                break;
            }
            /////////////////////
        case VCI_READ_REG:   // send REG value in VCI response
            {
#ifdef SOCLIB_NIC_DEBUG
                printf("[NIC][VCI_FSM] VCI_READ_REG\n");
#endif
                if ( p_vci.rspack.read() )
                    {
                        r_vci_fsm = VCI_IDLE;
                    }
                break;
            }
            /////////////////////
        case VCI_ERROR:   // send ERROR in VCI response
            {
#ifdef SOCLIB_NIC_DEBUG
                printf("[NIC][VCI_FSM] VCI_ERROR\n");
#endif
                if ( p_vci.rspack.read() )
                    {
                        r_vci_fsm = VCI_IDLE;
                    }
                break;
            }
        } // end switch tgt_fsm

    ///////////////////////////////////////////////////////////////////////////
    // This RX_G2S module makes the GMII to STREAM format conversion.
    // It checks the checksum, ans signals a possible error.
    // The input is the backend_rx object.
    // The output is the rx_fifo_stream, but this fifo is only used for
    // clock boundary handling, and should never be full,
    // as the consumer (RX_DES module) read all available bytes.
    ///////////////////////////////////////////////////////////////////////////

    // get data from PHY component
    bool    gmii_rx_dv;
    bool    gmii_rx_er;
    uint8_t gmii_rx_data;

#ifdef SOCLIB_PERF_NIC
    // Can delay the start/stop of the controller for DEBUG purpose
    if (nic_wait_start > 0 or nic_end > 800000)
        {
            gmii_rx_dv = false;
            gmii_rx_er = false;
            gmii_rx_data = 0;
            nic_wait_start -= 1;
        }
    else
        {
#endif

#ifdef SOCLIB_NIC_DEBUG
            // printf("[NIC][%s] Getting packet\n", __func__);
#endif
            r_backend_rx.get(&gmii_rx_dv,
                             &gmii_rx_er,
                             &gmii_rx_data);
#ifdef SOCLIB_NIC_DEBUG
            // printf("[NIC][%s] Got: dv = %d, er = %d, dt = %d\n", __func__, gmii_rx_dv, gmii_rx_er, gmii_rx_data);
#endif

#ifdef SOCLIB_PERF_NIC
            nic_end += 1;
        }
#endif

    // default values for fifo commands
    bool              rx_fifo_stream_write = false;
    uint16_t          rx_fifo_stream_wdata = 0;

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
                if (r_nic_on.read() and gmii_rx_dv and not gmii_rx_er ) // Power enable / start of packet / no error
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
                        r_rx_g2s_npkt_err = r_rx_g2s_npkt_err.read() + 1;
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
                        uint32_t crc_tmp_value = 0; // Hack to get rid of .get_new_value()
                        //compute CRC
                        crc_tmp_value = (r_rx_g2s_checksum.read() >> 4) ^ crc_table[(r_rx_g2s_checksum.read() ^ (r_rx_g2s_dt4.read() >> 0)) & 0x0F];
                        r_rx_g2s_checksum = (crc_tmp_value >> 4) ^ crc_table[(crc_tmp_value ^ (r_rx_g2s_dt4.read() >> 4)) & 0x0F];
                        r_rx_g2s_fsm      = RX_G2S_SOS;
                    }
                else
                    {
                        r_rx_g2s_npkt_err = r_rx_g2s_npkt_err.read() + 1;
                        r_rx_g2s_fsm      = RX_G2S_IDLE;
                    }
                break;
            }
            ////////////////
        case RX_G2S_SOS:    // write first byte in fifo_stream: SOS
            {
                if ( gmii_rx_dv and not gmii_rx_er ) // data valid / no error
                    {
                        uint32_t crc_tmp_value = 0; // Hack to get rid of .get_new_value()
                        //compute CRC
                        crc_tmp_value = (r_rx_g2s_checksum.read() >> 4) ^ crc_table[(r_rx_g2s_checksum.read() ^ (r_rx_g2s_dt4.read() >> 0)) & 0x0F];
                        r_rx_g2s_checksum = (crc_tmp_value >> 4) ^ crc_table[(crc_tmp_value ^ (r_rx_g2s_dt4.read() >> 4)) & 0x0F];
                        r_rx_g2s_fsm      = RX_G2S_LOOP;
                        rx_fifo_stream_write = true;
                        rx_fifo_stream_wdata = r_rx_g2s_dt5.read() | (STREAM_TYPE_SOS << 8);
                        r_rx_g2s_npkt = r_rx_g2s_npkt.read() + 1;
#ifdef SOCLIB_PERF_NIC
                        r_total_len_gmii = r_total_len_gmii.read() + 1;
#endif
                    }
                else
                    {
                        r_rx_g2s_npkt_err = r_rx_g2s_npkt_err.read() + 1;
                        r_rx_g2s_fsm      = RX_G2S_IDLE;
                    }
                break;
            }
            /////////////////
        case RX_G2S_LOOP:   // write one byte in fifo_stream : NEV
            {
                rx_fifo_stream_write = true;
                rx_fifo_stream_wdata = r_rx_g2s_dt5.read() | (STREAM_TYPE_NEV << 8);
                uint32_t crc_tmp_value = 0;
                //compute CRC
                crc_tmp_value = (r_rx_g2s_checksum.read() >> 4) ^ crc_table[(r_rx_g2s_checksum.read() ^ (r_rx_g2s_dt4.read() >> 0)) & 0x0F];
                r_rx_g2s_checksum = (crc_tmp_value >> 4) ^ crc_table[(crc_tmp_value ^ (r_rx_g2s_dt4.read() >> 4)) & 0x0F];
#ifdef SOCLIB_PERF_NIC
                r_total_len_gmii = r_total_len_gmii.read() + 1;
#endif

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
                        r_rx_g2s_fsm = RX_G2S_EXTD;
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

#ifdef SOCLIB_PERF_NIC
                r_total_len_gmii = r_total_len_gmii.read() + 1 ;
#endif

#ifdef SOCLIB_NIC_DEBUG
                printf("\nCHECK = %x\n",check);
                printf("RX : CHECKSUM_READ = %x\n",r_rx_g2s_checksum.read());
#endif

                if ( r_rx_g2s_checksum.read() == check )
                    {
#ifdef SOCLIB_NIC_DEBUG
                        printf("RX : CHECKSUM OK !\n");
#endif
                        rx_fifo_stream_write = true;
                        rx_fifo_stream_wdata = r_rx_g2s_dt5.read() | (STREAM_TYPE_EOS << 8);
                        r_rx_g2s_npkt_crc_success = r_rx_g2s_npkt_crc_success.read() + 1;
                    }
                else
                    {
#ifdef SOCLIB_NIC_DEBUG
                        printf("RX : CHECKSUM KO !\n");
#endif
                        rx_fifo_stream_write = true;
                        rx_fifo_stream_wdata = r_rx_g2s_dt5.read() | (STREAM_TYPE_ERR << 8);
                        r_rx_g2s_npkt_crc_fail = r_rx_g2s_npkt_crc_fail.read() + 1;
                    }

                if ( gmii_rx_dv and not gmii_rx_er ) // start of packet / no error
                    {
#ifdef SOCLIB_NIC_DEBUG
                        printf("RX_G2S_END avec RETOUR sur DELAY\n");
#endif
                        r_rx_g2s_fsm   = RX_G2S_DELAY;
                        r_rx_g2s_delay = 0;
                    }
                else
                    {
#ifdef SOCLIB_NIC_DEBUG
                        printf("RX_G2S_END avec RETOUR sur IDLE\n");
#endif
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
    // It is also charge of discarding input packets in four cases:
    // - if a packet is too small (64 - 4)B
    // - if a packet is too long (1518 - 4)B
    // - if a checksum error is reported by the RS_G2S FSM
    // - if there not space in the rx_fifo_multi
    ///////////////////////////////////////////////////////////i////////////////

    bool              rx_fifo_stream_read    = true;
    fifo_multi_wcmd_t rx_fifo_multi_wcmd     = FIFO_MULTI_WCMD_NOP;
    uint32_t          rx_fifo_multi_wdata    = 0;
    uint32_t          rx_fifo_multi_padding  = 0;  // only used for the last word
    bool              rx_des_packet_overflow = false;

    switch (r_rx_des_fsm.read())
        {
        case RX_DES_READ_0 :
            {
#ifdef SOCLIB_NIC_DEBUG
                //                printf("RX_DES_READ_0\n");
#endif
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
#ifdef SOCLIB_NIC_DEBUG
                printf("RX_DES_READ_1\n");
#endif
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
#ifdef SOCLIB_NIC_DEBUG
                printf("RX_DES_READ_2\n");
#endif
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
#ifdef SOCLIB_NIC_DEBUG
                printf("RX_DES_READ_3\n");
#endif
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
                                if (!r_rx_fifo_multi.wok())
                                    {
                                        r_rx_des_npkt_err_mfifo_full = r_rx_des_npkt_err_mfifo_full.read() + 1;
                                    }
                                r_rx_des_fsm = RX_DES_READ_0;
                            }
                    }
                break;
            }

        case RX_DES_READ_WRITE_0:
            {
#ifdef SOCLIB_NIC_DEBUG
                printf("RX_DES_READ_WRITE_0\n");
#endif
                uint16_t data;
                uint32_t type;
                uint32_t des_counter_bytes_tmp = 0;

                // write previous word into fifo_multi
                rx_fifo_multi_wcmd  = FIFO_MULTI_WCMD_WRITE;
                rx_fifo_multi_wdata = (uint32_t)(r_rx_des_data[0].read()      ) |
                    (uint32_t)(r_rx_des_data[1].read() << 8 ) |
                    (uint32_t)(r_rx_des_data[2].read() << 16) |
                    (uint32_t)(r_rx_des_data[3].read() << 24) ;
                /*rx_fifo_multi_wdata = (uint32_t)(r_rx_des_data[3].read()      ) |
                  (uint32_t)(r_rx_des_data[2].read() << 8 ) |
                  (uint32_t)(r_rx_des_data[1].read() << 16) |
                  (uint32_t)(r_rx_des_data[0].read() << 24) ;*/

                // Read new word
                data                = r_rx_fifo_stream.read();
                type                = (data >> 8) & 0x3;

                r_rx_des_data[0]	= (uint8_t)(data & 0xFF);
                r_rx_des_padding	= 3;


                if ( r_rx_fifo_stream.rok() )     // do nothing if we cannot read
                    {
                        des_counter_bytes_tmp = r_rx_des_counter_bytes.read() + 1;
                        r_rx_des_counter_bytes = r_rx_des_counter_bytes.read() + 1;

                        if ( des_counter_bytes_tmp > (MTU) )
                            {
#ifdef SOCLIB_NIC_DEBUG
                                printf("PACKET OVERFLOW\n");
#endif
                                rx_des_packet_overflow = true;
                            }

                        if (type == STREAM_TYPE_NEV and !rx_des_packet_overflow)
                            {
                                r_rx_des_fsm = RX_DES_READ_WRITE_1;
                            }

                        else if ( (type == STREAM_TYPE_EOS) and r_rx_fifo_multi.wok() and !rx_des_packet_overflow )
                            {
                                r_rx_des_fsm = RX_DES_WRITE_LAST;

                                if ( des_counter_bytes_tmp < (64-4) )
                                    {
#ifdef SOCLIB_NIC_DEBUG
                                        printf("PACKET TOO SMALL\n");
#endif
                                        r_rx_des_npkt_small = r_rx_des_npkt_small.read() + 1;
                                        r_rx_des_fsm = RX_DES_WRITE_CLEAR;
                                    }

                            }
                        else
                            {
                                if(rx_des_packet_overflow)
                                    r_rx_des_npkt_overflow = r_rx_des_npkt_overflow.read() + 1;

                                if(!r_rx_fifo_multi.wok())
                                    {
                                        r_rx_des_npkt_err_mfifo_full = r_rx_des_npkt_err_mfifo_full.read() + 1;
                                    }
                                r_rx_des_fsm = RX_DES_WRITE_CLEAR;
                            }
                    }
                break;
            }


       	case RX_DES_READ_WRITE_1:
            {
#ifdef SOCLIB_NIC_DEBUG
                printf("RX_DES_READ_WRITE_1\n");
#endif
                uint16_t data;
                uint32_t type;
                uint32_t des_counter_bytes_tmp = 0;

                data = r_rx_fifo_stream.read();
                type = (data >> 8) & 0x3;

                r_rx_des_data[1]	= (uint8_t)(data & 0xFF);
                r_rx_des_padding	= 2;

                if ( r_rx_fifo_stream.rok() )     // do nothing if we cannot read
                    {

                        des_counter_bytes_tmp = r_rx_des_counter_bytes.read() + 1;
                        r_rx_des_counter_bytes = r_rx_des_counter_bytes.read() + 1;

                        if ( des_counter_bytes_tmp > (MTU) )
                            {
#ifdef SOCLIB_NIC_DEBUG
                                printf("PACKET OVERFLOW\n");
#endif
                                rx_des_packet_overflow = true;
                            }

                        if (type == STREAM_TYPE_NEV and !rx_des_packet_overflow)
                            {
                                r_rx_des_fsm = RX_DES_READ_WRITE_2;
                            }

                        else if ( (type == STREAM_TYPE_EOS) and r_rx_fifo_multi.wok() and !rx_des_packet_overflow )
                            {
                                r_rx_des_fsm = RX_DES_WRITE_LAST;

                                if ( des_counter_bytes_tmp < (64-4) )
                                    {
#ifdef SOCLIB_NIC_DEBUG
                                        printf("PACKET TOO SMALL\n");
#endif
                                        r_rx_des_npkt_small = r_rx_des_npkt_small.read() + 1;
                                        r_rx_des_fsm = RX_DES_WRITE_CLEAR;
                                    }
                            }

                        else
                            {
                                if(rx_des_packet_overflow)
                                    r_rx_des_npkt_overflow = r_rx_des_npkt_overflow.read() + 1;
                                if(!r_rx_fifo_multi.wok())
                                    {
                                        r_rx_des_npkt_err_mfifo_full = r_rx_des_npkt_err_mfifo_full.read() + 1;
                                    }
                                r_rx_des_fsm = RX_DES_WRITE_CLEAR;
                            }
                    }
                break;
            }

       	case RX_DES_READ_WRITE_2:
            {
#ifdef SOCLIB_NIC_DEBUG
                printf("RX_DES_READ_WRITE_2\n");
#endif
                uint16_t data;
                uint32_t type;
                uint32_t des_counter_bytes_tmp = 0;

                data = r_rx_fifo_stream.read();
                type = (data >> 8) & 0x3;

                r_rx_des_data[2]	= (uint8_t)(data & 0xFF);

                r_rx_des_padding	= 1;


                if ( r_rx_fifo_stream.rok() )     // do nothing if we cannot read
                    {
                        des_counter_bytes_tmp = r_rx_des_counter_bytes.read() + 1;
                        r_rx_des_counter_bytes = r_rx_des_counter_bytes.read() + 1;

                        if ( des_counter_bytes_tmp > (MTU) )
                            {
#ifdef SOCLIB_NIC_DEBUG
                                printf("PACKET OVERFLOW\n");
#endif
                                rx_des_packet_overflow = true;
                            }

                        if (type == STREAM_TYPE_NEV and !rx_des_packet_overflow)
                            {
                                r_rx_des_fsm = RX_DES_READ_WRITE_3;
                            }

                        else if ( (type == STREAM_TYPE_EOS) and r_rx_fifo_multi.wok() and !rx_des_packet_overflow )
                            {
                                r_rx_des_fsm = RX_DES_WRITE_LAST;

                                if ( des_counter_bytes_tmp < (64-4) )
                                    {
#ifdef SOCLIB_NIC_DEBUG
                                        printf("PACKET TOO SMALL\n");
#endif
                                        r_rx_des_npkt_small = r_rx_des_npkt_small.read() + 1;
                                        r_rx_des_fsm = RX_DES_WRITE_CLEAR;
                                    }
                            }
                        else
                            {
                                if(rx_des_packet_overflow)
                                    r_rx_des_npkt_overflow = r_rx_des_npkt_overflow.read() + 1;
                                if(!r_rx_fifo_multi.wok())
                                    {
                                        r_rx_des_npkt_err_mfifo_full = r_rx_des_npkt_err_mfifo_full.read() + 1;
                                    }
                                r_rx_des_fsm = RX_DES_WRITE_CLEAR;
                            }
                    }
                break;
            }

        case RX_DES_READ_WRITE_3:
            {
                uint16_t data;
                uint32_t type;
                uint32_t des_counter_bytes_tmp = 0;

                data = r_rx_fifo_stream.read();
                type = (data >> 8) & 0x3;

                r_rx_des_data[3]	= (uint8_t)(data & 0xFF);

                r_rx_des_padding	= 0;


                if ( r_rx_fifo_stream.rok() )     // do nothing if we cannot read
                    {
                        des_counter_bytes_tmp = r_rx_des_counter_bytes.read() + 1;
                        r_rx_des_counter_bytes = r_rx_des_counter_bytes.read() + 1;

                        if ( des_counter_bytes_tmp > (MTU) )
                            {
#ifdef SOCLIB_NIC_DEBUG
                                printf("[NIC][RX_DES] RX_DES_READ_WRITE_3 : PACKET OVERFLOW\n");
#endif
                                rx_des_packet_overflow = true;
                            }


                        if ( (type == STREAM_TYPE_NEV) and r_rx_fifo_multi.wok() and !rx_des_packet_overflow )
                            {
                                r_rx_des_fsm = RX_DES_READ_WRITE_0;
                            }

                        else if ( (type == STREAM_TYPE_EOS) and r_rx_fifo_multi.wok() and !rx_des_packet_overflow )
                            {
                                r_rx_des_fsm = RX_DES_WRITE_LAST;

                                if ( des_counter_bytes_tmp < (64-4) )
                                    {

#ifdef SOCLIB_NIC_DEBUG
                                        printf("[NIC][RX_DES] RX_DES_READ_WRITE_3 : PACKET TOO SMALL : %d is < 60\n", des_counter_bytes_tmp);
#endif
                                        r_rx_des_npkt_small = r_rx_des_npkt_small.read() + 1;
                                        r_rx_des_fsm = RX_DES_WRITE_CLEAR;
                                    }

                            }
                        else
                            {
                                if(rx_des_packet_overflow)
                                    r_rx_des_npkt_overflow = r_rx_des_npkt_overflow.read() + 1;
                                if(!r_rx_fifo_multi.wok())
                                    {
                                        r_rx_des_npkt_err_mfifo_full = r_rx_des_npkt_err_mfifo_full.read() + 1;
                                    }
                                r_rx_des_fsm = RX_DES_WRITE_CLEAR;
                            }
                    }
                break;
            }

        case RX_DES_WRITE_LAST:     // write last word in rx_fifo_multi
            {
#ifdef SOCLIB_NIC_DEBUG
                printf("RX_DES_WRITE_LAST\n");
#endif
                rx_fifo_multi_wdata 	= (uint32_t)(r_rx_des_data[0].read()      ) |
                    (uint32_t)(r_rx_des_data[1].read() << 8 ) |
                    (uint32_t)(r_rx_des_data[2].read() << 16) |
                    (uint32_t)(r_rx_des_data[3].read() << 24) ;
                /*rx_fifo_multi_wdata 	= (uint32_t)(r_rx_des_data[3].read()      ) |
                  (uint32_t)(r_rx_des_data[2].read() << 8 ) |
                  (uint32_t)(r_rx_des_data[1].read() << 16) |
                  (uint32_t)(r_rx_des_data[0].read() << 24) ;*/

                rx_fifo_multi_wdata = rx_fifo_multi_wdata >>(r_rx_des_padding.read()<<3); // useless Bytes are set to 0
                rx_fifo_multi_wcmd  = FIFO_MULTI_WCMD_LAST;
                rx_fifo_multi_padding = r_rx_des_padding.read();
                r_rx_des_npkt_write_mfifo_success = r_rx_des_npkt_write_mfifo_success.read() + 1;
                rx_des_packet_overflow = false;
                r_rx_des_fsm = RX_DES_READ_0;
                break;
            }

	    case RX_DES_WRITE_CLEAR :
            {
#ifdef SOCLIB_NIC_DEBUG
                printf("RX_DES_WRITE_CLEAR\n");
#endif
                rx_fifo_multi_wcmd  = FIFO_MULTI_WCMD_CLEAR;
                rx_des_packet_overflow = false;
                r_rx_des_npkt_err_in_des = r_rx_des_npkt_err_in_des.read() + 1;
                r_rx_des_fsm        = RX_DES_READ_0;
            }
        } // end swich rx_des_fsm

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
    uint32_t             rx_channel_wdata      = 0;
    uint32_t             rx_channel_padding    = 0;
    rx_channel_wcmd_t    rx_channel_wcmd[m_channels];

    // initializing last declared variable
    for (size_t j = 0; j < m_channels; j++)
        rx_channel_wcmd[j] = RX_CHANNEL_WCMD_NOP;


    switch( r_rx_dispatch_fsm.read() )
        {
        case RX_DISPATCH_IDLE:  // ready to start a new packet transfer
            {
#ifdef SOCLIB_NIC_DEBUG
                //                printf("RX_DISPATCH_IDLE\n");
#endif
                if ( r_rx_dispatch_bp.read() )  // previously allocated to bp_fifo
                    {
                        if ( r_rx_fifo_multi.rok() ) r_rx_dispatch_bp = false;
                    }
                else                            // previously allocated to rx_fifo
                    {
                        if ( r_bp_fifo_multi.rok() )
                            {
                                r_rx_dispatch_bp = true;
                            }
                    }

                if ( r_bp_fifo_multi.rok() or r_rx_fifo_multi.rok() ) // packet available
                    {
                        r_rx_dispatch_fsm = RX_DISPATCH_GET_PLEN;
                    }
                break;
            }
        case RX_DISPATCH_GET_PLEN: // get packet length from fifo
            {
#ifdef SOCLIB_NIC_DEBUG
                printf("[NIC][RX_DISPATCH_GET_PLEN] Entering the state \n");
#endif
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
#ifdef SOCLIB_NIC_DEBUG
                printf("[NIC][RX_DISPATCH_READ_FIRST] Entering the state\n");
#endif
                if ( r_rx_dispatch_bp.read() )
                    {
                        bp_fifo_multi_rcmd       = FIFO_MULTI_RCMD_READ;
                    }
                else
                    {
                        rx_fifo_multi_rcmd       = FIFO_MULTI_RCMD_READ;
                    }
                r_rx_dispatch_words = r_rx_dispatch_words.read() - 1;
                r_rx_dispatch_fsm   = RX_DISPATCH_CHANNEL_SELECT;

                break;
            }
        case RX_DISPATCH_CHANNEL_SELECT:  // check destination MAC address
            {
                // we read the second data word, without modifying the fifo state
#ifdef SOCLIB_NIC_DEBUG
                printf("[NIC][RX_DISPATCH_CHANNEL_SELECT] Entering the state\n");
#endif
                unsigned int    data_ext;
                bool            found = false;

                if ( r_rx_dispatch_bp.read() )  data_ext = (r_bp_fifo_multi.data() & 0xFFFF0000)>>16;
                else                            data_ext = (r_rx_fifo_multi.data() & 0xFFFF0000)>>16;

                // check if broadcast mode is enable and if the mac addr is a broadcast addr
                if ((r_rx_dispatch_dt0.read() == 0xFFFFFFFF)  and (data_ext == 0xFFFF) and (r_broadcast_enable.read() == 1))
                    {
#ifdef SOCLIB_NIC_DEBUG
                        printf("[NIC][RX_DISPATCH_CHANNEL_SELECT] Broadcast detected\n");
#endif
                        r_rx_dispatch_broadcast = 1;
                        r_rx_dispatch_fsm = RX_DISPATCH_GET_CHANNEL_BROADCAST;
                    }
                else
                    {
                        for ( size_t k = 0 ; (k < m_channels) and not found ; k++ )
                            {
                                // check if the current channel is actived and if this mac addr is set
                                if (((r_channel_active_channels.read() >> k ) & 0x1) and ((r_channel_mac_addr_set.read() >> k) & 0x1))
                                    {
                                        if ( (r_channel_mac_4[k].read() == r_rx_dispatch_dt0.read() ) and
                                             (r_channel_mac_2[k].read() == data_ext) )
                                            {
                                                found                 = true;
                                                r_rx_dispatch_channel = k;
                                            }
                                    }
                                else
                                    {
#ifdef SOCLIB_NIC_DEBUG
                                        if ((r_channel_active_channels.read() >> k) & 0x1)
                                            printf("[NIC][RX_DISPATCH_CHANNEL_SELECT] PKT SKIP : channel disable or not init\n");
                                        else
                                            printf("[NIC][RX_DISPATCH_CHANNEL_SELECT] PKT SKIP : channel mac addr not set\n");
#endif
                                    }
                            }
                        if (found)
                            r_rx_dispatch_fsm = RX_DISPATCH_GET_WOK;
                        else
                            {
                                r_rx_dispatch_npkt_skip_adrmac_fail = r_rx_dispatch_npkt_skip_adrmac_fail.read() + 1;
                                r_rx_dispatch_fsm = RX_DISPATCH_PACKET_SKIP;
                            }
                    }
                if ( r_rx_dispatch_bp.read() )
                    {
                        bp_fifo_multi_rcmd       = FIFO_MULTI_RCMD_READ;
                    }
                else
                    {
                        rx_fifo_multi_rcmd       = FIFO_MULTI_RCMD_READ;
                    }
                r_rx_dispatch_words = r_rx_dispatch_words.read() - 1;
                break;
            }
        case RX_DISPATCH_PACKET_SKIP:	// clear an unexpected packet in source fifo
            {
#ifdef SOCLIB_NIC_DEBUG
                printf("[NIC][RX_DISPATCH_PACKET_SKIP] PACKET SKIP !\n");
#endif

                if ( r_rx_dispatch_bp.read() )
                    bp_fifo_multi_rcmd = FIFO_MULTI_RCMD_SKIP;
                else
                    rx_fifo_multi_rcmd = FIFO_MULTI_RCMD_SKIP;

                r_rx_dispatch_fsm = RX_DISPATCH_IDLE;
                break;
            }
        case RX_DISPATCH_GET_WOK: // test if there is an open container in selected channel (Unicast mode)
            {
#ifdef SOCLIB_NIC_DEBUG
                printf("[NIC][RX_DISPATCH_GET_WOK] Entering the state\n");
#endif
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
        case RX_DISPATCH_GET_CHANNEL_BROADCAST : // test if there is at least an open container
                                                 // we select wich channels will be write
            {
                uint32_t sel_wok = r_rx_sel_channel_wok.read();
                uint32_t plen    = r_rx_dispatch_plen.read();
                uint32_t space;
                uint32_t sel_space_timeout_ok = r_rx_sel_space_timeout_ok.read();

                for ( size_t k = 0 ; (k < m_channels); k++ )
                    {
                        space   = r_rx_channel[k]->space();

                        if(((r_channel_active_channels.read()>>k)&0x1) and ((r_channel_mac_addr_set.read()>>k)&0x1)) // test channel active
                            // @ MAC set
                            {
                                if (r_rx_channel[k]->wok())
                                    sel_wok = sel_wok | (1<<k);
                                else
                                    sel_wok = sel_wok &~ (1<<k);
                            }
                        else
                            sel_wok = sel_wok &~ (1<<k);

                        if ((plen <= space) && ((((int32_t)plen >> 2) + 1) < r_rx_channel[k]->get_r_timer())) // test enough space and time
                            sel_space_timeout_ok = sel_space_timeout_ok | (1<<k);
                        else
                            sel_space_timeout_ok = sel_space_timeout_ok &~ (1<<k);
                    }

                r_rx_sel_space_timeout_ok = sel_space_timeout_ok;
                r_rx_sel_channel_wok = sel_wok;

                if ( (sel_wok == 0) || (sel_space_timeout_ok == 0) )  // if no channel ready to be write => skip packet
                    {
                        r_rx_dispatch_fsm = RX_DISPATCH_PACKET_SKIP;
                        r_rx_dispatch_npkt_wchannel_fail = r_rx_dispatch_npkt_wchannel_fail.read() + 1;
                    }
                else
                    r_rx_dispatch_fsm = RX_DISPATCH_CLOSE_CONT;
                break;
            }
        case RX_DISPATCH_GET_SPACE: // test available space in selected channel (Unicast mode)
            {
                uint32_t channel = r_rx_dispatch_channel.read();
                uint32_t space   = r_rx_channel[channel]->space();
                uint32_t plen    = r_rx_dispatch_plen.read();

                // Will write only if :
                // - There is enough space
                // - There is enough time to write it
                if ((plen <= space) && ((((int32_t)plen >> 2) + 1) < r_rx_channel[channel]->get_r_timer()))
                    {
                        r_rx_dispatch_fsm = RX_DISPATCH_CHECK_MAC_SRC;
                    }
                else
                    r_rx_dispatch_fsm = RX_DISPATCH_CLOSE_CONT;
                break;
            }
        case RX_DISPATCH_CLOSE_CONT:  // Not enough space or time to write: close container
            {
                uint32_t channel;
                uint32_t sel_wok;
                uint32_t sel_space_timeout_ok;

#ifdef SOCLIB_NIC_DEBUG
                printf("[NIC][RX_DISPATCH_CLOSE_CONT] Entering the state\n");
#endif
                // check if broadcast mode is enable and if the packet is detected as broadcast
                if( (r_rx_dispatch_broadcast == 1) and (r_broadcast_enable.read() == 1))
                    {
                        for ( size_t k = 0 ; (k < m_channels); k++ )
                            {
                                sel_wok = (r_rx_sel_channel_wok.read()>>k)&0x1;
                                sel_space_timeout_ok = (r_rx_sel_space_timeout_ok.read()>>k)&0x1;
                                if((sel_wok == 1) and (sel_space_timeout_ok == 0))
                                    {
#ifdef SOCLIB_NIC_DEBUG
                                        printf("[NIC][RX_DISPATCH_CLOSE_CONT] channel %d is close\n",k);
#endif
                                        rx_channel_wcmd[k]   = RX_CHANNEL_WCMD_CLOSE;
                                    }
                            }
                        r_rx_dispatch_fsm = RX_DISPATCH_CHECK_MAC_SRC;
                    }
                else // Unicast mode
                    {
                        channel = r_rx_dispatch_channel.read();
                        rx_channel_wcmd[channel]   = RX_CHANNEL_WCMD_CLOSE;
                        r_rx_dispatch_fsm = RX_DISPATCH_GET_WOK;
                    }
                break;
            }
        case RX_DISPATCH_CHECK_MAC_SRC : // test Source MAC address
            {
                uint32_t data_mac_4_src;
                uint32_t data_mac_2_src;
                uint32_t tmp;
                uint32_t sel_wok = r_rx_sel_channel_wok.read();

#ifdef SOCLIB_NIC_DEBUG
                printf("[NIC][RX_DISPATCH_CHECK_MAC_SRC] Entering the state\n");
#endif
                if (r_rx_dispatch_bp.read())
                    {
                        data_mac_4_src           = r_bp_fifo_multi.data();
                    }
                else
                    {
                        data_mac_4_src           = r_rx_fifo_multi.data();
                    }

                data_mac_2_src = data_mac_4_src & 0x0000FFFF;
                tmp = ((r_rx_dispatch_dt0.read()&0x0000FFFF)<<16)|(((data_mac_4_src&0xFFFF0000)>>16));
                for(size_t k = 0 ; (k < m_channels); k++ )
                    {
                        if ( r_rx_dispatch_bp.read() )
                            {
                                if((r_channel_mac_4[k].read() == tmp) and (r_channel_mac_2[k].read() == data_mac_2_src)) // In Broadcast mode
                                    // If Src MAC address = MAC[k]
                                    // This channel "k" will not receive the pkt
                                    {
                                        sel_wok = sel_wok &~ (1<<k);
                                    }
                            }
                    }
                r_rx_sel_channel_wok = sel_wok;
                r_rx_dispatch_fsm    = RX_DISPATCH_READ_WRITE;
                break;
            }
        case RX_DISPATCH_READ_WRITE:    // read a new word from fifo and
                                        // write previous word to channel
            {
#ifdef SOCLIB_NIC_DEBUG
                printf("[NIC][RX_DISPATCH_READ_WRITE] Entering the state\n");
#endif

                uint32_t channel;
                uint32_t sel_wok;
                uint32_t sel_space_timeout_ok;

                // check if broadcast mode is enable and if the packet is detected as broadcast
                if( (r_rx_dispatch_broadcast == 1) and (r_broadcast_enable.read() == 1) )
                    {
#ifdef SOCLIB_PERF_NIC
                        r_total_len_rx_chan = r_total_len_rx_chan.read() + 4;
#endif
                        for ( size_t k = 0 ; (k < m_channels); k++ )
                            {
                                sel_wok = (r_rx_sel_channel_wok.read()>>k)&0x1;
                                sel_space_timeout_ok = (r_rx_sel_space_timeout_ok.read()>>k)&0x1;

                                if((sel_wok == 1) and (sel_space_timeout_ok == 1))
                                    {
#ifdef SOCLIB_NIC_DEBUG
                                        printf("[NIC][RX_DISPATCH_READ_WRITE] channel %d is writed\n",k);
#endif
                                        rx_channel_wcmd[k] = RX_CHANNEL_WCMD_WRITE;
                                    }
                            }
                    }
                else
                    {
                        channel = r_rx_dispatch_channel.read();
                        rx_channel_wcmd[channel]     = RX_CHANNEL_WCMD_WRITE;
#ifdef SOCLIB_PERF_NIC
                        r_total_len_rx_chan = r_total_len_rx_chan.read() + 4;
#endif
                    }
                rx_channel_wdata    = r_rx_dispatch_data.read();
                if ( r_rx_dispatch_bp.read() )
                    {
                        if      ( r_rx_dispatch_words.read() > 1 )  bp_fifo_multi_rcmd = FIFO_MULTI_RCMD_READ;
                        else if ( r_rx_dispatch_words.read() == 1 ) bp_fifo_multi_rcmd = FIFO_MULTI_RCMD_LAST;
                        else                                        bp_fifo_multi_rcmd = FIFO_MULTI_RCMD_NOP;
                    }
                else
                    {
                        if      ( r_rx_dispatch_words.read() > 1 )  rx_fifo_multi_rcmd = FIFO_MULTI_RCMD_READ;
                        else if ( r_rx_dispatch_words.read() == 1 ) rx_fifo_multi_rcmd = FIFO_MULTI_RCMD_LAST;
                        else                                        rx_fifo_multi_rcmd = FIFO_MULTI_RCMD_NOP;
                    }
                r_rx_dispatch_words = r_rx_dispatch_words.read() - 1;
                if (r_rx_dispatch_words.read() == 0)
                    {
                        r_rx_dispatch_fsm   = RX_DISPATCH_WRITE_LAST;
                        break;
                    }
                break;
            }
        case RX_DISPATCH_WRITE_LAST:  // write last word to selected channel
            {
#ifdef SOCLIB_NIC_DEBUG
                printf("[NIC][RX_DISPATCH_WRITE_LAST] Entering the state\n");
#endif
                uint32_t plen = r_rx_dispatch_plen.read();
                uint32_t channel;
                uint32_t sel_wok;
                uint32_t sel_space_timeout_ok;

                if ( (plen & 0x3) == 0 )    rx_channel_padding  = 0;
                else                        rx_channel_padding = 4 - (plen & 0x3);

                if( (r_rx_dispatch_broadcast == 1) and (r_broadcast_enable.read() == 1) )
                    {
                        for ( size_t k = 0 ; (k < m_channels); k++ )
                            {
#ifdef SOCLIB_PERF_NIC
                                r_total_len_rx_chan = r_total_len_rx_chan.read() + (4 - rx_channel_padding) ;
#endif
                                sel_wok = (r_rx_sel_channel_wok.read()>>k)&0x1;
                                sel_space_timeout_ok = (r_rx_sel_space_timeout_ok.read()>>k)&0x1;
                                if((sel_wok == 1) and (sel_space_timeout_ok == 1))
                                    {
                                        rx_channel_wcmd[k] = RX_CHANNEL_WCMD_LAST;
#ifdef SOCLIB_NIC_DEBUG
                                        printf("[NIC][RX_DISPATCH_WRITE_LAST] channel[%d] is write LAST\n", k);
#endif
                                    }
                            }
                        r_rx_dispatch_broadcast = 0;
                    }
                else
                    {
#ifdef SOCLIB_PERF_NIC
                        r_total_len_rx_chan = r_total_len_rx_chan.read() + (4 - rx_channel_padding);
#endif
                        channel = r_rx_dispatch_channel.read();
                        rx_channel_wcmd[channel]     = RX_CHANNEL_WCMD_LAST;
                    }

                rx_channel_wdata    = r_rx_dispatch_dt0.read();
                r_rx_dispatch_npkt_wchannel_success = r_rx_dispatch_npkt_wchannel_success.read() + 1;
                r_rx_dispatch_fsm   = RX_DISPATCH_IDLE;
                break;
            }
        } // end switch r_rx_dispatch_fsm
    // test if READ or LAST RCMD on rx_fifo_multi or bp_fifo_multi to fill the pipeline
    if (rx_fifo_multi_rcmd == FIFO_MULTI_RCMD_READ or rx_fifo_multi_rcmd == FIFO_MULTI_RCMD_LAST or bp_fifo_multi_rcmd == FIFO_MULTI_RCMD_READ or bp_fifo_multi_rcmd == FIFO_MULTI_RCMD_LAST)
        {
            if ( r_rx_dispatch_bp.read() )
                r_rx_dispatch_dt0 = r_bp_fifo_multi.data();
            else
                r_rx_dispatch_dt0 = r_rx_fifo_multi.data();


            r_rx_dispatch_data = r_rx_dispatch_dt0.read() ;
        }


    /////////////////////////////////////////////////////////////////////
    // The TX_DISPATCH FSM performs the actual transfer of
    // all packet from a tx_channel[k] container to tx_fifo or bp_fifo,
    // depending on the MAC address.
    // - the by-pass fifo is a multi-buffer fifo (typically 4 Kbytes).
    // - the tx_fifo is a simple fifo (10 bits width / 2 slots depth).
    // It implements a round-robin priority between the channels or TDM
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

    // test if TDM Scheduler enable
    if(r_tx_tdm_enable.read() == 1)
        {
            // test if nic power enabled
            if(r_nic_on.read() ==  0)
                // ?????????????
                r_tx_tdm_timer = r_tx_chan_tdm_timer[0].read();
            else
                {
                    r_tx_tdm_timer = r_tx_tdm_timer.read() - 1;
                    // test if (tdm_timer == 0) to select the new priority channel
                    if( r_tx_tdm_timer.read() == 1 )
                        {
                            uint32_t chan_sel_tdm = r_tx_chan_sel_tdm.read();
                            chan_sel_tdm = ((chan_sel_tdm + 1) % m_channels);
                            for (size_t i = chan_sel_tdm; i < (chan_sel_tdm + m_channels) ; i++)
                                {
                                    // the disabled channels are not taken into account
                                    if (((r_channel_active_channels.read()>>(i%m_channels))&0x1) and ((r_channel_mac_addr_set.read()>>(i%m_channels))&0x1))
                                        {
                                            chan_sel_tdm = i%m_channels;
                                            break;
                                        }
                                }
                            r_tx_tdm_timer = r_tx_chan_tdm_timer[chan_sel_tdm].read();
                            r_tx_chan_sel_tdm = chan_sel_tdm;
                        }
                }
        }

    switch( r_tx_dispatch_fsm.read() )
        {
            //////////////////////
        case TX_DISPATCH_IDLE:  // ready to start a new packet transfer
            {
                bool pwen = r_nic_on.read();
                if(r_tx_tdm_enable.read() == 1)
                    {
                        uint32_t chan_sel_tdm = r_tx_chan_sel_tdm.read();
                        if( r_tx_channel[chan_sel_tdm]->rok() && pwen)
                            {
                                r_tx_dispatch_channel = chan_sel_tdm;
                                // if it's a new container we must know the number of packet
                                if (r_tx_dispatch_packets[chan_sel_tdm].read() == 0)
                                    {
                                        r_tx_dispatch_fsm     = TX_DISPATCH_GET_NPKT;
                                    }
                                // if the container has already begun to be emptied => get_plen
                                else
                                    {
                                        r_tx_dispatch_fsm     = TX_DISPATCH_GET_PLEN;
                                    }
                                break;
                            }
                    }
                else // RR scheduler
                    {
                        for ( size_t x = 0 ; x < m_channels ; x++ )
                            {
                                size_t k = (x + 1 + r_tx_dispatch_channel.read()) % m_channels;

                                if (r_tx_channel[k]->rok() && pwen)
                                    {
#ifdef SOCLIB_NIC_DEBUG
                                        printf("[NIC][TX_DISPATCH_IDLE] 1 channel not empty\n");
#endif
                                        r_tx_dispatch_channel = k;
                                        r_tx_dispatch_fsm     = TX_DISPATCH_GET_NPKT;
                                        break;
                                    }
                            }
                    }
                break;
            }
            //////////////////////////
        case TX_DISPATCH_GET_NPKT: // get packet number from tx_channel
            {
                uint32_t    channel   = r_tx_dispatch_channel.read();
                uint32_t    npkt      = r_tx_channel[channel]->npkt();

                r_tx_dispatch_packets[channel] = npkt;
#ifdef SOCLIB_NIC_DEBUG
                printf("[NIC][TX_DISPATCH_GET_NPKT] channel[%d] has %d packets ready\n", channel, npkt);
#endif
                if ((npkt == 0) or (npkt > 66))
                    r_tx_dispatch_fsm = TX_DISPATCH_RELEASE_CONT;
                else
                    r_tx_dispatch_fsm = TX_DISPATCH_GET_PLEN;
                break;
            }
        case TX_DISPATCH_GET_PLEN: // get packet length from tx_channel
            {
                uint32_t channel      = r_tx_dispatch_channel.read();
                uint32_t plen         = r_tx_channel[channel]->plen();

                r_tx_dispatch_broadcast = 0;
                r_tx_dispatch_first_bytes_pckt = 1; // first bytes of current pkt
                r_tx_dispatch_interne = 0;          // internal transmit if 1
                r_tx_dispatch_bytes   = plen & 0x3;
                r_tx_dispatch_pipe_count = 2;       // pipeline depth

                if ((plen & 0x3) == 0)
                    {
                        r_tx_dispatch_words = plen >> 2;
                    }
                else
                    {
                        r_tx_dispatch_words = (plen >> 2) + 1;
                    }

                // #ifdef SOCLIB_NIC_DEBUG
                //                 printf("TX_DISPATCH_GET_PLEN : %d %d %d\n",plen,r_tx_dispatch_bytes.get_new_value(),r_tx_dispatch_words.get_new_value() );
                // #endif

                if (plen < MIN_TRAME_SIZE) // pkt too small
                    {
#ifdef SOCLIB_NIC_DEBUG
                        printf("[NIC][TX_DISPATCH_GET_PLEN] PKT SKIP : packet too small : plen = %d (0x%04x)\n", plen, plen);
#endif
                        r_tx_dispatch_fsm = TX_DISPATCH_SKIP_PKT; // maybe should go to TX_DISPATCH_RELEASE_CONT
                        r_tx_npkt_small = r_tx_npkt_small.read() + 1;
                    }
                else
                    if (plen > MTU) // pkt too long
                        {
#ifdef SOCLIB_NIC_DEBUG
                            printf("[NIC][TX_DISPATCH_GET_PLEN] PKT SKIP : packet too long : plen = %d (0x%04x)\n", plen, plen);
#endif
                            r_tx_dispatch_fsm = TX_DISPATCH_SKIP_PKT; // maybe should go to TX_DISPATCH_RELEASE_CONT
                            r_tx_npkt_overflow = r_tx_npkt_overflow.read() + 1;
                        }
                    else
                        if (!(((r_channel_active_channels.read() >> channel) & 0x1) and ((r_channel_mac_addr_set.read() >> channel) & 0x1)))
                            {
#ifdef SOCLIB_NIC_DEBUG
                                if ((r_channel_active_channels.read() >> channel) & 0x1)
                                    printf("[NIC][TX_DISPATCH_GET_PLEN] PKT SKIP : channel disable or not init\n");
                                else
                                    printf("[NIC][TX_DISPATCH_GET_PLEN] PKT SKIP : channel mac addr not set\n");
#endif
                                r_tx_dispatch_fsm = TX_DISPATCH_SKIP_PKT; // maybe should go to TX_DISPATCH_RELEASE_CONT
                            }
                // check if TDM scheduler enable , if the packet can be written in the time, and if the priority channel has not change
                        else
                            {
                                if ( (r_tx_tdm_enable.read() == 1) && ( (r_tx_tdm_timer.read() < (plen+4+12)) || (r_tx_dispatch_channel.read() != r_tx_chan_sel_tdm.read()) ) )
                                    {
                                        r_tx_dispatch_fsm = TX_DISPATCH_IDLE;
                                    }
                                else
                                    {
                                        r_tx_dispatch_fsm  = TX_DISPATCH_READ_FIRST;
                                    }
                            }
                break;
            }

        case TX_DISPATCH_SKIP_PKT :
            {
                uint32_t  channel = r_tx_dispatch_channel.read();
                uint32_t  packets = r_tx_dispatch_packets[channel].read();

                tx_channel_rcmd     = TX_CHANNEL_RCMD_SKIP;
                r_tx_dispatch_packets[channel] = packets - 1;

                if (packets == 1)
                    r_tx_dispatch_fsm = TX_DISPATCH_RELEASE_CONT;
                else
                    r_tx_dispatch_fsm = TX_DISPATCH_GET_PLEN;
#ifdef SOCLIB_NIC_DEBUG
                printf("[NIC][TX_DISPATCH_SKIP_PKT] SKIP PACKET !\n");
#endif
                break;
            }

        case TX_DISPATCH_READ_FIRST: // read first word from tx_channel
            {
                tx_channel_rcmd     = TX_CHANNEL_RCMD_READ;
                r_tx_dispatch_words = r_tx_dispatch_words.read() - 1;
                r_tx_dispatch_fsm   = TX_DISPATCH_FIFO_SELECT;
#ifdef SOCLIB_PERF_NIC
                r_total_len_tx_chan = r_total_len_tx_chan.read() + 4;
#endif
                break;
            }
        case TX_DISPATCH_FIFO_SELECT: // check DST MAC address
            {
                // we read the second data word, without modifying the channel state
                uint32_t        channel  = r_tx_dispatch_channel.read();
                uint32_t        data_mac_2_dest = be16toh(r_tx_channel[channel]->data() & 0x0000FFFF);
                uint32_t        data_mac_4_dest = be32toh(r_tx_dispatch_dt0.read());
                uint32_t        found    = r_tx_dispatch_interne.read();

                tx_channel_rcmd = TX_CHANNEL_RCMD_READ;
#ifdef SOCLIB_PERF_NIC
                r_total_len_tx_chan = r_total_len_tx_chan.read() + 4;
#endif
                r_tx_dispatch_words = r_tx_dispatch_words.read() - 1;

                // In any case, we will get to the next state at the next cycle
                r_tx_dispatch_fsm = TX_DISPATCH_CHECK_MAC_ADDR_SRC;

                // check if broadcast pkt
                if (( data_mac_4_dest == 0xFFFFFFFF) and (data_mac_2_dest == 0xFFFF) and (r_broadcast_enable.read() == 1))
                    {
                        r_tx_dispatch_broadcast = 1;
                    }
                else
                    {
#ifdef SOCLIB_NIC_DEBUG
                        printf("[NIC][TX_DISPATCH_FIFO_SELECT] destination addr mac 4 from pkt  = 0x%08x\n", data_mac_4_dest);
                        printf("[NIC][TX_DISPATCH_FIFO_SELECT] destination addr mac 2 from pkt = 0x%04x\n", data_mac_2_dest);
#endif
                        // we run through each channel to find if we get it to be swtiched internaly
                        for (size_t k = 0; (k < m_channels) and (found == 0); k++)
                            {
                                if ((r_channel_mac_4[k].read() == data_mac_4_dest) and (r_channel_mac_2[k].read() == data_mac_2_dest))
                                    {
#ifdef SOCLIB_NIC_DEBUG
                                        printf("[NIC][TX_DISPATCH_FIFO_SELECT] internal switching\n");
#endif
                                        found = 1;
                                        r_tx_dispatch_channel_interne_send = k;
                                        r_tx_dispatch_interne = found;

                                        if (k == channel)
                                            {
#ifdef SOCLIB_NIC_DEBUG
                                                printf("[NIC][TX_DISPATCH_FIFO_SELECT] PKT SKIP : sending to itself same channel\n");
#endif
                                                r_tx_dispatch_fsm = TX_DISPATCH_SKIP_PKT;
                                            }
                                    }
                            }
                    }
                break;
            }

        case TX_DISPATCH_CHECK_MAC_ADDR_SRC : // check if SRC MAC address of pkt = MAC address of sender
            {
                uint32_t channel    = r_tx_dispatch_channel.read();
                uint32_t tmp = r_tx_channel[channel]->data();
                uint32_t data_mac_4_src = (be32toh(r_tx_dispatch_dt0.read() & 0xFFFF0000) << 16) | be16toh(tmp & 0x0000FFFF);
                uint32_t data_mac_2_src = be16toh((tmp & 0xFFFF0000) >> 16);
                uint32_t target = r_tx_dispatch_channel_interne_send.read();

#ifdef SOCLIB_PERF_NIC
                r_total_len_tx_chan = r_total_len_tx_chan.read() + 4;
#endif

                tx_channel_rcmd     = TX_CHANNEL_RCMD_READ;
                r_tx_dispatch_data  = r_tx_dispatch_dt1.read();
                r_tx_dispatch_words = r_tx_dispatch_words.read() - 1;

#ifdef SOCLIB_NIC_DEBUG
                printf("[NIC][TX_DISPATCH_CHECK_MAC_ADDR_SRC] addr mac 4 channel[%d] = 0x%08x and addr mac 4 src = 0x%08x\n", channel, r_channel_mac_4[channel].read(), data_mac_4_src);
                printf("[NIC][TX_DISPATCH_CHECK_MAC_ADDR_SRC] addr mac 2 channel[%d] = 0x%04x and addr mac 2 src = 0x%04x\n", channel, r_channel_mac_2[channel].read(), data_mac_2_src);
#endif
                if ((r_channel_mac_4[channel].read() ==  data_mac_4_src)
                    and (r_channel_mac_2[channel].read() == data_mac_2_src))
                    {
                        if ((r_tx_dispatch_broadcast == 1) and (r_broadcast_enable.read() == 1))
                            {
#ifdef SOCLIB_NIC_DEBUG
                                printf("[NIC][TX_DISPATCH_CHECK_MAC_ADDR_SRC] sending in broadcast mode\n");
#endif
                                r_tx_dispatch_fsm = TX_DISPATCH_WRITE_B0_TX;
                            }
                        else
                            if (r_tx_dispatch_interne == 1)
                                {
#ifdef SOCLIB_NIC_DEBUG
                                    printf("[NIC][TX_DISPATCH_CHECK_MAC_ADDR_SRC] sending in inside mode\n");
#endif
                                    r_tx_channel_to_channel[channel][target] = r_tx_channel_to_channel[channel][target].read() + 1;
                                    r_tx_dispatch_fsm = TX_DISPATCH_READ_WRITE_BP;
                                }
                            else
                                {
#ifdef SOCLIB_NIC_DEBUG
                                    printf("[NIC][TX_DISPATCH_CHECK_MAC_ADDR_SRC] sending in outside mode\n");
#endif
                                    r_tx_dispatch_fsm = TX_DISPATCH_WRITE_B0_TX;
                                }
                    }
                else
                    {
#ifdef SOCLIB_NIC_DEBUG
                        printf("[NIC][TX_DISPATCH_CHECK_MAC_ADDR_SRC] PKT SKIP : Source MAC addr check failed\n");
#endif
                        r_tx_dispatch_addr_mac_src_fail = r_tx_dispatch_addr_mac_src_fail.read() + 1;
                        r_tx_dispatch_fsm = TX_DISPATCH_SKIP_PKT;
                    }
                break;
            }
        case TX_DISPATCH_READ_WRITE_BP: // write previous data in bp_fifo
            // and read a new data from selected channel
            {
#ifdef SOCLIB_NIC_DEBUG
                printf("[NIC][TX_DISPATCH_READ_WRITE_BP] Entering the state\n");
#endif
                uint32_t  words   = r_tx_dispatch_words.read();

                if ( r_bp_fifo_multi.wok() )
                    {
                        bp_fifo_multi_wcmd    = FIFO_MULTI_WCMD_WRITE;
                        bp_fifo_multi_wdata   = r_tx_dispatch_data.read();
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
#ifdef SOCLIB_NIC_DEBUG
                printf("[NIC][TX_DISPATCH_WRITE_LAST_BP] Entering the state\n");
#endif
                uint32_t  channel      = r_tx_dispatch_channel.read();
                uint32_t  packets      = r_tx_dispatch_packets[channel].read();
                uint32_t  bytes        = r_tx_dispatch_bytes.read();
                uint32_t  pipe_count   = r_tx_dispatch_pipe_count.read();

                if (r_bp_fifo_multi.wok())
                    {
                        if (pipe_count == 0) // we need to empty the pipeline
                            {
                                bp_fifo_multi_wcmd    = FIFO_MULTI_WCMD_LAST;
                                bp_fifo_multi_wdata   = r_tx_dispatch_data.read();
                                if ( bytes == 0 )
                                    bp_fifo_multi_padding = 0;
                                else
                                    bp_fifo_multi_padding = 4 - bytes;
                                r_tx_dispatch_packets[channel] = packets - 1;
                                r_tx_npkt = r_tx_npkt.read() + 1;
                                r_tx_dispatch_pipe_count = 2;
                                r_tx_dispatch_fsm = TX_DISPATCH_IFG;
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
                uint32_t channel      = r_tx_dispatch_channel.read();
                uint32_t words        = r_tx_dispatch_words.read();
                uint32_t bytes        = r_tx_dispatch_bytes.read();
                uint32_t packets      = r_tx_dispatch_packets[channel].read();
                uint32_t pipe_count = r_tx_dispatch_pipe_count.read();

                if( (r_tx_dispatch_broadcast.read() == 1) and (r_broadcast_enable.read() == 1) )
                    {
                        if ( r_bp_fifo_multi.wok() and r_tx_fifo_stream.wok() )
                            {
                                bp_fifo_multi_wcmd    = FIFO_MULTI_WCMD_WRITE;
                                bp_fifo_multi_wdata   = r_tx_dispatch_data.read();

                                if(pipe_count == 0) // we need to empty the pipeline
                                    {
                                        bp_fifo_multi_wcmd    = FIFO_MULTI_WCMD_LAST;
                                        if ( bytes == 0 )
                                            bp_fifo_multi_padding = 0;
                                        else
                                            bp_fifo_multi_padding = 4 - bytes;
                                    }
                            }
                    }

                if ( r_tx_fifo_stream.wok() and (r_bp_fifo_multi.wok() or (r_tx_dispatch_broadcast.read() == 0)) )
                    {
                        tx_fifo_stream_write = true;
                        if ( (r_tx_dispatch_first_bytes_pckt.read() == 1))
                            {
#ifdef SOCLIB_NIC_DEBUG
                                printf("[NIC][TX_DISPATCH_WRITE_B0_TX] Start Of Stream\n");
#endif
                                tx_fifo_stream_wdata = (r_tx_dispatch_data.read() & 0x000000FF) | (STREAM_TYPE_SOS<<8);
                                r_tx_dispatch_first_bytes_pckt = 0;
                            }
                        else
                            {
                                tx_fifo_stream_wdata = (r_tx_dispatch_data.read() & 0x000000FF) | (STREAM_TYPE_NEV<<8);
                            }
                        if ( (words == 0) and (bytes == 1) and (pipe_count == 0) )   // last byte to write in tx_fifo
                            {
                                tx_fifo_stream_wdata         = (r_tx_dispatch_data.read() & 0x000000FF) | (STREAM_TYPE_EOS <<8);
                                r_tx_dispatch_packets[channel] = packets - 1;
                                r_tx_npkt = r_tx_npkt.read() + 1;
                                r_tx_dispatch_fsm = TX_DISPATCH_IFG;
                            }
                        else
                            {
                                r_tx_dispatch_fsm = TX_DISPATCH_WRITE_B1_TX;
                            }
                    }
                break;
            }
        case TX_DISPATCH_WRITE_B1_TX: // write byte 1 in tx_fifo
            {
                uint32_t channel      = r_tx_dispatch_channel.read();
                uint32_t words        = r_tx_dispatch_words.read();
                uint32_t bytes        = r_tx_dispatch_bytes.read();
                uint32_t packets      = r_tx_dispatch_packets[channel].read();
                uint32_t pipe_count   = r_tx_dispatch_pipe_count.read();

                if ( r_tx_fifo_stream.wok() )
                    {
                        tx_fifo_stream_write = true;
                        tx_fifo_stream_wdata = ((r_tx_dispatch_data.read() >> 8) & 0x000000FF) | (STREAM_TYPE_NEV<<8);
                        if ( (words == 0) and (bytes == 2) and(pipe_count == 0))   // last byte to write in tx_fifo
                            {
                                tx_fifo_stream_wdata         = ((r_tx_dispatch_data.read()>>8) & 0x000000FF) | (STREAM_TYPE_EOS <<8);
                                r_tx_dispatch_packets[channel] = packets - 1;
                                r_tx_npkt = r_tx_npkt.read() + 1;
                                r_tx_dispatch_fsm = TX_DISPATCH_IFG;
                            }
                        else
                            {
                                r_tx_dispatch_fsm = TX_DISPATCH_WRITE_B2_TX;
                            }
                    }
                break;
            }
        case TX_DISPATCH_WRITE_B2_TX: // write byte 2 in tx_fifo
            {
                uint32_t channel      = r_tx_dispatch_channel.read();
                uint32_t words        = r_tx_dispatch_words.read();
                uint32_t bytes        = r_tx_dispatch_bytes.read();
                uint32_t packets      = r_tx_dispatch_packets[channel].read();
                uint32_t pipe_count   = r_tx_dispatch_pipe_count.read();

                if ( r_tx_fifo_stream.wok() )
                    {
                        tx_fifo_stream_write = true;
                        tx_fifo_stream_wdata = ((r_tx_dispatch_data.read() >> 16) & 0x000000FF) | (STREAM_TYPE_NEV<<8);
                        if ( (words == 0) and (bytes == 3) and(pipe_count==0))   // last byte to write in tx_fifo
                            {
                                tx_fifo_stream_wdata         = ((r_tx_dispatch_data.read()>>16) & 0x000000FF) | (STREAM_TYPE_EOS <<8);
                                r_tx_dispatch_packets[channel] = packets - 1;
                                r_tx_npkt = r_tx_npkt.read() + 1;
                                r_tx_dispatch_fsm = TX_DISPATCH_IFG;
                            }
                        else
                            {
                                r_tx_dispatch_fsm = TX_DISPATCH_READ_WRITE_TX;
                            }
                    }
                break;
            }
        case TX_DISPATCH_READ_WRITE_TX: // write byte 3 in tx_fifo
            // and read word from selected channel
            // if the current word is not the last
            {
                uint32_t channel      = r_tx_dispatch_channel.read();
                uint32_t words        = r_tx_dispatch_words.read();
                uint32_t packets      = r_tx_dispatch_packets[channel].read();
                uint32_t pipe_count   = r_tx_dispatch_pipe_count.read();
                uint32_t bytes        = r_tx_dispatch_bytes.read();

                if (r_tx_fifo_stream.wok())
                    {
                        tx_fifo_stream_write = true;
                        tx_fifo_stream_wdata = ((r_tx_dispatch_data.read() >> 24) & 0x000000FF) | (STREAM_TYPE_NEV<<8);
                        if (words == 0) // last byte to write in tx_fifo
                            {
                                if (pipe_count == 0)
                                    {
                                        tx_fifo_stream_wdata         = ((r_tx_dispatch_data.read()>>24) & 0x000000FF) | (STREAM_TYPE_EOS <<8);
                                        r_tx_dispatch_packets[channel] = packets - 1;
                                        r_tx_npkt = r_tx_npkt.read() + 1;
                                        r_tx_dispatch_fsm = TX_DISPATCH_IFG;
                                    }
                                else
                                    {
                                        if (pipe_count == 2)
                                            r_tx_dispatch_data = r_tx_dispatch_dt1.read();
                                        else
                                            if (pipe_count == 1)
                                                r_tx_dispatch_data = r_tx_dispatch_dt0.read();
                                        r_tx_dispatch_pipe_count = r_tx_dispatch_pipe_count.read() - 1;
                                        r_tx_dispatch_fsm = TX_DISPATCH_WRITE_B0_TX;
                                    }
                            }
                        else
                            if (words == 1) // last word to read in tx_channel
                                {
                                    tx_channel_rcmd     = TX_CHANNEL_RCMD_LAST;
#ifdef SOCLIB_PERF_NIC
                                    if (bytes == 0)
                                        r_total_len_tx_chan = r_total_len_tx_chan.read() + 4;
                                    else
                                        r_total_len_tx_chan = r_total_len_tx_chan.read() + bytes;
#endif
                                    r_tx_dispatch_data  = r_tx_dispatch_dt1.read();
                                    r_tx_dispatch_words = words - 1;
                                    r_tx_dispatch_fsm   = TX_DISPATCH_WRITE_B0_TX;
                                }
                            else
                                if (words > 1) // not the last word to read in tx_channel
                                    {
                                        tx_channel_rcmd     = TX_CHANNEL_RCMD_READ;
#ifdef SOCLIB_PERF_NIC
                                        r_total_len_tx_chan = r_total_len_tx_chan.read() + 4;
#endif
                                        r_tx_dispatch_data  = r_tx_dispatch_dt1.read();
                                        r_tx_dispatch_words = words - 1;
                                        r_tx_dispatch_fsm   = TX_DISPATCH_WRITE_B0_TX;
                                    }
                    }
                break;
            }
        case TX_DISPATCH_IFG : // Inter Frame Gap State
            {
#ifdef SOCLIB_NIC_DEBUG
                printf("[NIC][TX_DISPATCH_IFG] Waiting %d cycles\n", r_tx_dispatch_ifg.read());
#endif
                r_tx_dispatch_ifg = r_tx_dispatch_ifg.read() - 1;
                if (r_tx_dispatch_ifg.read() == 1)
                    {
                        uint32_t channel      = r_tx_dispatch_channel.read();
                        uint32_t packets      = r_tx_dispatch_packets[channel].read();
                        r_tx_dispatch_ifg = IFG; // reset the IFG value

                        // if no more packets
                        if (packets == 0)
                            {
#ifdef SOCLIB_PERF_NIC
                                r_total_len_tx_chan = r_total_len_tx_chan.read();
#endif
                                r_tx_dispatch_fsm = TX_DISPATCH_RELEASE_CONT;
                            }
                        else
                            {
#ifdef SOCLIB_PERF_NIC
                                r_total_len_tx_chan = r_total_len_tx_chan.read();
#endif
                                r_tx_dispatch_fsm = TX_DISPATCH_GET_PLEN;
                            }
                    }
                break;
            }
        case TX_DISPATCH_RELEASE_CONT: // release the container in tx_channel
            {
#ifdef SOCLIB_NIC_DEBUG
                printf("[NIC][TX_DISPATCH_RELEASE_CONT] Entering the state\n");
#endif
                tx_channel_rcmd   = TX_CHANNEL_RCMD_RELEASE;
                r_tx_dispatch_fsm = TX_DISPATCH_IDLE;
                break;
            }
        } // end switch tx_dispatch_fsm

    // if tx_channel_rcmd = READ or LAST we fill the pipeline
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
    // The output is the r_backend_tx module.
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

                        assert ( (type == STREAM_TYPE_SOS) and
                                 "ERROR in VCI_MULTI_NIC : illegal type received in TX_S2G_IDLE");

                        tx_fifo_stream_read = true;
                        r_tx_s2g_fsm        = TX_S2G_WRITE_DATA;
                        r_tx_s2g_data       = data & 0xFF;
                        r_tx_s2g_checksum = 0x00000000; // reset checksum
                    }
                r_backend_tx.put(false, false,0);
                break;
            }
            //////////////////////
        case TX_S2G_WRITE_DATA:     // write one data byte into gmii_tx
                                    // and read next byte from fifo_stream
            {
                if ( r_tx_fifo_stream.rok() )
                    {
                        uint32_t data = r_tx_fifo_stream.read();
                        uint32_t type = (data >> 8) & 0x3;

                        assert ( (type != STREAM_TYPE_SOS) and (type != STREAM_TYPE_ERR) and
                                 "ERROR in VCI_MULTI_NIC : illegal type received in TX_S2G_WRITE_DATA");

                        tx_fifo_stream_read = true;
                        r_tx_s2g_data       = data & 0xFF;

#ifdef SOCLIB_NIC_DEBUG
                        printf("[NIC][TX_S2G_WRITE_DATA] writing : 0x%02x\n", r_tx_s2g_data.read());
#endif

                        uint32_t crc_tmp_value = 0;
                        //compute CRC
                        crc_tmp_value = (r_tx_s2g_checksum.read() >> 4) ^ crc_table[(r_tx_s2g_checksum.read() ^ (r_tx_s2g_data.read() >> 0)) & 0x0F];
                        r_tx_s2g_checksum = (crc_tmp_value >> 4) ^ crc_table[(crc_tmp_value ^ (r_tx_s2g_data.read() >> 4)) & 0x0F];
#ifdef SOCLIB_PERF_NIC
                        r_total_len_tx_gmii = r_total_len_tx_gmii.read() + 1;
#endif
                        if ( type == STREAM_TYPE_EOS )
                            {
                                r_tx_s2g_fsm = TX_S2G_WRITE_LAST_DATA;
                            }
                    }
                else
                    {
                        assert (false and
                                "ERROR in VCI_MULTI_NIC : tx_fifo should not be empty");
                    }
                r_backend_tx.put(true,false,r_tx_s2g_data.read());
                break;
            }
            ////////////////////
        case TX_S2G_WRITE_LAST_DATA :
            {
                uint32_t crc_tmp_value = 0;

                r_tx_s2g_fsm = TX_S2G_WRITE_CS;
                r_backend_tx.put(true,false,r_tx_s2g_data.read());

                // compute CRC
                crc_tmp_value = (r_tx_s2g_checksum.read() >> 4) ^ crc_table[(r_tx_s2g_checksum.read() ^ (r_tx_s2g_data.read() >> 0)) & 0x0F];
                r_tx_s2g_checksum = (crc_tmp_value >> 4) ^ crc_table[(crc_tmp_value ^ (r_tx_s2g_data.read() >> 4)) & 0x0F];

#ifdef SOCLIB_PERF_NIC
                r_total_len_tx_gmii = r_total_len_tx_gmii.read() + 1;
#endif
                break;
            }


            ////////////////////
        case TX_S2G_WRITE_CS:       // write one cs byte into gmii_out
            {
                uint8_t gmii_data;

                if ( r_tx_s2g_index.read() == 0 )
                    {
#ifdef SOCLIB_NIC_DEBUG
                        printf("[NIC][TX_S2G_WRITE_DATA] writing CRC32 : 0x%08x\n", r_tx_s2g_checksum.read());
#endif
                        gmii_data      = r_tx_s2g_checksum.read() & 0xFF;
                        r_tx_s2g_index = 1;
                    }
                else
                    if ( r_tx_s2g_index.read() == 1 )
                        {
                            gmii_data      = (r_tx_s2g_checksum.read() >> 8) & 0xFF;
                            r_tx_s2g_index = 2;
                        }
                    else
                        if ( r_tx_s2g_index.read() == 2 )
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
                            }

                r_backend_tx.put(true, false, gmii_data);
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

    ////////////////////////////////////////////////////////////////////////////
    // update stream_fifos
    ////////////////////////////////////////////////////////////////////////////
    r_rx_fifo_stream.update( rx_fifo_stream_read,
                             rx_fifo_stream_write,
                             rx_fifo_stream_wdata );

    r_tx_fifo_stream.update( tx_fifo_stream_read,
                             tx_fifo_stream_write,
                             tx_fifo_stream_wdata );

    ////////////////////////////////////////////////////////////////////////////
    // update rx_channels
    ////////////////////////////////////////////////////////////////////////////
    for ( size_t k = 0 ; k < m_channels ; k++ )
        {
            rx_channel_rcmd_t read;
            rx_channel_wcmd_t write;

            write = rx_channel_wcmd[k];

            if ( r_vci_channel.read() == k )            read  = rx_channel_rcmd;
            else                                        read  = RX_CHANNEL_RCMD_NOP;

            r_rx_channel[k]->update( write,
                                     read,
                                     rx_channel_wdata,
                                     rx_channel_padding );
        }

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
            // print_trace(1);
            break;
        }

    case VCI_READ_REG:
        {
            p_vci.cmdack = false;
            p_vci.rspval = true;
            p_vci.rdata  = read_register((uint32_t)p_vci.address.read());
            p_vci.rerror = vci_param::ERR_NORMAL;
            p_vci.rsrcid = r_vci_srcid.read();
            p_vci.rtrdid = r_vci_trdid.read();
            p_vci.rpktid = r_vci_pktid.read();
            p_vci.reop   = true;
            // print_trace(1);
            break;
        }
    case VCI_ERROR:
        {
            p_vci.cmdack = false;
            p_vci.rspval = true;
            p_vci.rdata  = 0;
            p_vci.rerror = vci_param::ERR_GENERAL_DATA_ERROR;
            p_vci.rsrcid = r_vci_srcid.read();
            p_vci.rtrdid = r_vci_trdid.read();
            p_vci.rpktid = r_vci_pktid.read();
            p_vci.reop   = true;
            break;
        }

    } // end switch vci_fsm

} // end genMore()

/////////////////////////
tmpl(void)::print_trace(uint32_t option)
{
    const char* vci_state_str[] =
        {
            "VCI_IDLE",
            "VCI_WRITE_TX_BURST",
            "VCI_READ_RX_BURST",
            "VCI_WRITE_TX_LAST",
            "VCI_WRITE_REG",
            "VCI_READ_REG",
            "VCI_ERROR",
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
            "RX_DISPATCH_GET_CHANNEL_BROADCAST",
            "RX_DISPATCH_CLOSE_CONT",
            "RX_DISPATCH_GET_SPACE",
            "RX_DISPATCH_CHECK_MAC_SRC",
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
            "TX_DISPATCH_IFG",
            "TX_DISPATCH_RELEASE_CONT",
        };
    const char* tx_s2g_state_str[] =
        {
            "TX_S2G_IDLE",
            "TX_S2G_WRITE_DATA",
            "TX_S2G_WRITE_LAST_DATA",
            "TX_S2G_WRITE_CS",
        };
    if (option == 1) // used to print all internals registers
        {
#ifdef SOCLIB_PERF_NIC
            std::cout << "r_total_len_gmii    : " << r_total_len_gmii.read()    << std::endl;
            std::cout << "r_total_len_rx_chan : " << r_total_len_rx_chan.read() << std::endl;
            std::cout << "r_total_len_tx_chan : " << r_total_len_tx_chan.read() << std::endl;
            std::cout << "r_total_len_tx_gmii : " << r_total_len_tx_gmii.read() << std::endl;
#endif
            std::cout << "r_broadcast_enable  : " << r_broadcast_enable.read()  << std::endl;
            std::cout << "r_nic_on            : " << r_nic_on.read()            << std::endl;
            std::cout << "r_rx_dispatch_broadcast : " << r_rx_dispatch_broadcast.read() << std::endl;
            std::cout << "r_channel_active_channels : " <<std::hex<< r_channel_active_channels.read() << std::endl;
            std::cout << "r_channel_mac_addr_set : " <<std::hex<< r_channel_mac_addr_set.read() << std::endl;
            std::cout << "r_rx_sel_channel_wok : " << std::hex << r_rx_sel_channel_wok.read() << std::endl;
            std::cout << "r_rx_sel_space_timeout_ok : " <<std::hex<< r_rx_sel_space_timeout_ok.read() <<std::dec<< std::endl;
            std::cout << "r_vci_fsm : " << r_vci_fsm.read() << std::endl;
            std::cout << "r_vci_channel : " << r_vci_channel.read() << std::endl;
            std::cout << "r_vci_ptw : " << r_vci_ptw.read() << std::endl;
            std::cout << "r_vci_ptr : " << r_vci_ptr.read() << std::endl;
            std::cout << "r_vci_nwords : " << r_vci_nwords.read() << std::endl;
            std::cout << "r_rx_g2s_fsm : " << r_rx_g2s_fsm.read() << std::endl;
            std::cout << "r_rx_g2s_checksum : " <<std::hex<< r_rx_g2s_checksum.read() << std::endl;
            std::cout << "r_rx_g2s_dt0 : " << (unsigned int)r_rx_g2s_dt0.read() << std::endl;
            std::cout << "r_rx_g2s_dt1 : " << (unsigned int)r_rx_g2s_dt1.read() << std::endl;
            std::cout << "r_rx_g2s_dt2 : " << (unsigned int)r_rx_g2s_dt2.read() << std::endl;
            std::cout << "r_rx_g2s_dt3 : " << (unsigned int)r_rx_g2s_dt3.read() << std::endl;
            std::cout << "r_rx_g2s_dt4 : " << (unsigned int)r_rx_g2s_dt4.read() << std::endl;
            std::cout << "r_rx_g2s_dt5 : " << (unsigned int)r_rx_g2s_dt5.read() <<std::dec<< std::endl;
            std::cout << "r_rx_g2s_delay : " << r_rx_g2s_delay.read() << std::endl;
            std::cout << "r_rx_g2s_npkt : " << r_rx_g2s_npkt.read() << std::endl;
            std::cout << "r_rx_g2s_npkt_crc_success : " << r_rx_g2s_npkt_crc_success.read() << std::endl;
            std::cout << "r_rx_g2s_npkt_crc_fail : " << r_rx_g2s_npkt_crc_fail.read() << std::endl;
            std::cout << "r_rx_g2s_npkt_err : " << r_rx_g2s_npkt_err.read() << std::endl;
            std::cout << "r_rx_des_fsm : " << r_rx_des_fsm.read() << std::endl;
            std::cout << "r_rx_des_counter_bytes : " << r_rx_des_counter_bytes.read() << std::endl;
            std::cout << "r_rx_des_padding : " << r_rx_des_padding.read() << std::endl;
            std::cout << "r_rx_des_npkt_err_in_des : " << r_rx_des_npkt_err_in_des.read() << std::endl;
            std::cout << "r_rx_des_npkt_write_mfifo_success : " << r_rx_des_npkt_write_mfifo_success.read() << std::endl;
            std::cout << "r_rx_des_npkt_small : " << r_rx_des_npkt_small.read() << std::endl;
            std::cout << "r_rx_des_npkt_overflow : " << r_rx_des_npkt_overflow.read() << std::endl;
            std::cout << "r_rx_des_npkt_err_mfifo_full : " << r_rx_des_npkt_err_mfifo_full.read() << std::endl;
            std::cout << "r_rx_dispatch_fsm : " << r_rx_dispatch_fsm.read() << std::endl;
            std::cout << "r_rx_dispatch_channel : " << r_rx_dispatch_channel.read() << std::endl;
            std::cout << "r_rx_dispatch_bp : " << r_rx_dispatch_bp.read() << std::endl;
            std::cout << "r_rx_dispatch_plen : " << r_rx_dispatch_plen.read() << std::endl;
            std::cout << "r_rx_dispatch_dt0 : " << r_rx_dispatch_dt0.read() << std::endl;
            std::cout << "r_rx_dispatch_data : " << r_rx_dispatch_data.read() << std::endl;
            std::cout << "r_rx_dispatch_words : " << r_rx_dispatch_words.read() << std::endl;
            std::cout << "r_rx_dispatch_npkt_skip_adrmac_fail : " << r_rx_dispatch_npkt_skip_adrmac_fail.read() << std::endl;
            std::cout << "r_rx_dispatch_npkt_wchannel_success : " << r_rx_dispatch_npkt_wchannel_success.read() << std::endl;
            std::cout << "r_rx_dispatch_npkt_wchannel_fail : " << r_rx_dispatch_npkt_wchannel_fail.read() << std::endl;
            std::cout << "r_tx_dispatch_fsm : " << r_tx_dispatch_fsm.read() << std::endl;
            std::cout << "r_tx_dispatch_channel : " << r_tx_dispatch_channel.read() << std::endl;
            std::cout << "r_tx_dispatch_data : " << r_tx_dispatch_data.read() << std::endl;
            std::cout << "r_tx_dispatch_words : " << r_tx_dispatch_words.read() << std::endl;
            std::cout << "r_tx_dispatch_bytes : " << r_tx_dispatch_bytes.read() << std::endl;
            std::cout << "r_tx_dispatch_first_bytes_pckt : " << r_tx_dispatch_first_bytes_pckt.read() << std::endl;
            std::cout << "r_tx_npkt : " << r_tx_npkt.read() << std::endl;
            std::cout << "r_tx_npkt_overflow : " << r_tx_npkt_overflow.read() << std::endl;
            std::cout << "r_tx_npkt_small : " << r_tx_npkt_small.read() << std::endl;
            std::cout << "r_tx_dispatch_addr_mac_src_fail : " << r_tx_dispatch_addr_mac_src_fail.read() << std::endl;
            std::cout << "r_tx_dispatch_dt0 : " << r_tx_dispatch_dt0.read() << std::endl;
            std::cout << "r_tx_dispatch_dt1 : " << r_tx_dispatch_dt1.read() << std::endl;
            std::cout << "r_tx_dispatch_dt2 : " << r_tx_dispatch_dt2.read() << std::endl;
            std::cout << "r_tx_dispatch_interne : " << r_tx_dispatch_interne.read() << std::endl;
            std::cout << "r_tx_dispatch_pipe_count : " << r_tx_dispatch_pipe_count.read() << std::endl;
            std::cout << "r_tx_dispatch_broadcast : " << r_tx_dispatch_broadcast.read() << std::endl;
            std::cout << "r_tx_dispatch_channel_interne_send : " << r_tx_dispatch_channel_interne_send.read() << std::endl;
            std::cout << "r_tx_dispatch_ifg : " << r_tx_dispatch_ifg.read() << std::endl;
            std::cout << "r_tx_s2g_fsm : " << r_tx_s2g_fsm.read() << std::endl;
            std::cout << "r_tx_s2g_checksum : " << r_tx_s2g_checksum.read() << std::endl;
            std::cout << "r_tx_s2g_data : " << (unsigned int)r_tx_s2g_data.read() << std::endl;
            std::cout << "r_tx_s2g_index : " << r_tx_s2g_index.read() << std::endl;
            std::cout << "r_tx_tdm_enable : " << r_tx_tdm_enable.read() << std::endl;
            std::cout << "r_tx_tdm_timer : " << r_tx_tdm_timer.read() << std::endl;
            std::cout << "r_tx_chan_sel_tdm : " << r_tx_chan_sel_tdm.read() << std::endl;

            for(size_t i = 0; i < 4; i++)
                std::cout << "r_rx_des_data ["<< i <<"] : " << (unsigned int)r_rx_des_data[i].read() << std::endl;

            for(size_t i = 0; i < m_channels; i++)
                {
                    std::cout << "r_channel_mac_4 ["  << i <<"] : "<<std::hex<< r_channel_mac_4[i].read() <<std::dec<< std::endl;
                    std::cout << "r_channel_mac_2 ["  << i <<"] : "<<std::hex<< r_channel_mac_2[i].read() <<std::dec<< std::endl;
                    std::cout << "r_tx_dispatch_packets ["  << i <<"] : "<< r_tx_dispatch_packets[i].read() << std::endl;
                    std::cout << "r_tx_chan_tdm_timer ["  << i <<"] : "<< r_tx_chan_tdm_timer[i].read() << std::endl;
                    for(size_t j = 0; j < m_channels; j++)
                        std::cout << "r_tx_channel_to_channel ["  << i <<"] ["<< j <<"] : " << r_tx_channel_to_channel[i][j].read() << std::endl;
                }
        }
    else
        {
            std::cout << "MULTI_NIC " << name() << " : "
                      << vci_state_str[r_vci_fsm.read()]                 << " | "
                      << rx_g2s_state_str[r_rx_g2s_fsm.read()]           << " | "
                      << rx_des_state_str[r_rx_des_fsm.read()]           << " | "
                      << rx_dispatch_state_str[r_rx_dispatch_fsm.read()] << " | "
                      << tx_dispatch_state_str[r_tx_dispatch_fsm.read()] << " | "
                      << tx_s2g_state_str[r_tx_s2g_fsm.read()]           << std::endl;
        }
} // end print_trace()




/////////////////////////
tmpl(int32_t)::open_tap_fd()
{
    int32_t tap_fd = -1;

#ifdef SOCLIB_NIC_DEBUG
    printf("[NIC][%s] Opening TAP fd\n", __func__);
#endif

    tap_fd = open("/dev/net/tun", O_RDWR);

    if (tap_fd < 0) // error
        {
            std::cerr << name() << ": Unable to open /dev/net/tun" << std::endl;
        }
    else // we can continue
        {
            int flags = fcntl(tap_fd, F_GETFL, 0);
            fcntl(tap_fd, F_SETFL, flags | O_NONBLOCK);

#ifdef SOCLIB_NIC_DEBUG
            printf("[NIC][%s] Converting TAP fd to TAP interface\n", __func__);
#endif
            memset((void*)&m_tap_ifr, 0, sizeof(m_tap_ifr));
            m_tap_ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
            // strncpy(m_tap_ifr.ifr_name, if_name.c_str(), IFNAMSIZ); // we don't have name right now, let the kernel decide

            if (ioctl(tap_fd, TUNSETIFF, (void *) &m_tap_ifr) < 0)
                {
                    close(tap_fd);
                    tap_fd = -1;
                    std::cerr << name() << ": Unable to setup tap interface, check privileges."
#ifdef __linux__
                              << " (try: sudo setcap cap_net_admin=eip ./system.x)"
#endif
                              << std::endl;
                    return -1;
                }
            else
                {
                    std::cout << name() << ": TAP interface succesfully created: " << m_tap_ifr.ifr_name << "\n";
                }
        }

    return tap_fd;
} // end open_tap_fd




////////////////////////////////////////////////////////////////////
tmpl(/**/)::VciMultiNic( sc_core::sc_module_name 		        name,
                         const soclib::common::IntTab 		    &tgtid,
                         const soclib::common::MappingTable     &mt,
                         const size_t 				            channels,
                         const char*                            rx_file_pathname,
                         const char*                            tx_file_pathname,
                         const size_t 				            timeout,
                         const char                             *macaddr)
           : caba::BaseModule(name),
#ifdef SOCLIB_PERF_NIC
           r_total_len_gmii("r_total_len_gmii"),
           r_total_len_rx_chan("r_total_len_rx_chan"),
           r_total_len_tx_chan("r_total_len_tx_chan"),
           r_total_len_tx_gmii("r_total_len_tx_gmii"),
#endif
           r_broadcast_enable("r_broadcast_enable"),
           r_nic_on("r_nic_on"),
           r_rx_dispatch_broadcast("r_rx_dispatch_broadcast"),

           r_channel_active_channels("r_channel_active_channels"),
           r_channel_mac_addr_set("r_channel_mac_addr_set"),
           r_rx_sel_channel_wok ("r_rx_sel_channel_wok"),
           r_rx_sel_space_timeout_ok("r_rx_sel_space_timeout_ok"),

           r_vci_fsm("r_vci_fsm"),
           r_vci_srcid("r_vci_srcid"),
           r_vci_trdid("r_vci_trdid"),
           r_vci_pktid("r_vci_pktid"),
           r_vci_wdata("r_vci_wdata"),
           r_vci_ptw("r_vci_ptw"),
           r_vci_ptr("r_vci_ptr"),
           r_vci_nwords("r_vci_nwords"),
           r_vci_address("r_vci_address"),

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
           r_rx_des_npkt_err_in_des("r_rx_des_npkt_drop_in_des"),
           r_rx_des_npkt_write_mfifo_success("r_rx_des_npkt_write_mfifo_success"),
           r_rx_des_npkt_small("r_rx_des_npkt_small"),
           r_rx_des_npkt_overflow("r_rx_des_npkt_overflow"),
           r_rx_des_npkt_err_mfifo_full("r_rx_des_npkt_err_mfifo_full"),

           r_rx_dispatch_fsm("r_rx_dispatch_fsm"),
           r_rx_dispatch_channel("r_rx_dispatch_channel"),
           r_rx_dispatch_bp("r_rx_dispatch_bp"),
           r_rx_dispatch_plen("r_rx_dispatch_plen"),
           r_rx_dispatch_dt0("r_rx_dispatch_dt0"),
           r_rx_dispatch_data("r_rx_dispatch_data"),
           r_rx_dispatch_words("r_rx_dispatch_words"),
           r_rx_dispatch_npkt_skip_adrmac_fail("r_rx_dispatch_npkt_skip_adrmac_fail"),
           r_rx_dispatch_npkt_wchannel_success("r_rx_dispatch_npkt_wchannel_success"),
           r_rx_dispatch_npkt_wchannel_fail("r_rx_dispatch_npkt_wchannel_fail"),

           r_tx_dispatch_fsm("r_tx_dispatch_fsm"),
           r_tx_dispatch_channel("r_tx_dispatch_channel"),
           r_tx_dispatch_data("r_tx_dispatch_data"),
           //r_tx_dispatch_packets("r_tx_dispatch_packets"),
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
           r_tx_dispatch_broadcast("r_tx_dispatch_broadcast"),
           r_tx_dispatch_channel_interne_send("r_tx_dispatch_channel_interne_send"),
           r_tx_dispatch_ifg("r_tx_dispatch_ifg"),
           r_tx_s2g_fsm("r_tx_s2g_fsm"),
           r_tx_s2g_checksum("r_tx_s2g_checksum"),
           r_tx_s2g_data("r_tx_s2g_data"),
           r_tx_s2g_index("r_tx_s2g_index"),

           r_tx_tdm_enable("r_tx_tdm_enable"),
           r_tx_tdm_timer("r_tx_tdm_timer"),
           r_tx_chan_sel_tdm("r_tx_chan_sel_tdm"),

           r_rx_fifo_stream("r_rx_fifo_stream", 2),      // 2 slots of one byte
           r_rx_fifo_multi("r_rx_fifo_multi", 32, 32),   // 1024 slots of one word
           r_tx_fifo_stream("r_tx_fifo_stream", 2),      // 2 slots of one byte
           r_bp_fifo_multi("r_bp_fifo_multi", 32, 32),   // 1024 slots of one word

           r_backend_rx("r_backend_rx", rx_file_pathname, IFG),
           r_backend_tx("r_backend_tx", tx_file_pathname),

           m_segment(mt.getSegment(tgtid)),
           m_channels(channels),

           p_clk("p_clk"),
           p_resetn("p_resetn"),
           p_vci("p_vci"),
           p_rx_irq(soclib::common::alloc_elems<sc_core::sc_out<bool> >("p_rx_irq", channels)),
           p_tx_irq(soclib::common::alloc_elems<sc_core::sc_out<bool> >("p_tx_irq", channels))
{
    int32_t             tmp_fd = -1;

#ifdef SOCLIB_NIC_DEBUG
    printf("[NIC][%s] Entering constructor\n", __func__);
#endif

    assert( (vci_param::B == 4) and
            "VCI_MULTI_NIC error : The VCI DATA field must be 32 bits");

    assert( (channels <= 8)  and
            "VCI_MULTI_NIC error : The number of channels cannot be larger than 8");

    //////////////////////////////////////////////////
    // We get a TAP fd if no path file have been given
    //////////////////////////////////////////////////
#ifdef SOCLIB_NIC_DEBUG
    //    printf("[NIC][%s] rx_file_pathname = %s and tx_file_pathname = %s\n", __func__, rx_file_pathname, tx_file_pathname);
#endif
    // if (rx_file_pathname == NULL or tx_file_pathname == NULL)
    tmp_fd = open_tap_fd();

    if (tmp_fd < 0) // error during fd oppening, we leave
        {
            // assert( (tmp_fd >= 0)  and
            //         "VCI_MULTI_NIC error : Problem opening the TAP interface");
            std::cerr << name << ": Problem opening the TAP interface " << std::endl;
            exit(-1);
        }

    // if (rx_file_pathname == NULL)
    //     {
    r_backend_rx.set_fd(tmp_fd);
    r_backend_rx.set_ifr(&m_tap_ifr);
    // }

    // if (tx_file_pathname == NULL)
    //     {
    r_backend_tx.set_fd(tmp_fd);
    r_backend_tx.set_ifr(&m_tap_ifr);
    // }
    ////////////////////////////////////////////////// End TAP

    r_rx_channel = new NicRxChannel*[channels];
    r_tx_channel = new NicTxChannel*[channels];

    r_channel_mac_4       = new sc_signal<uint32_t>[channels];
    r_channel_mac_2       = new sc_signal<uint32_t>[channels];

    // Set the default MAC addr
    for ( size_t k = 0; k < MAC_ADDR_LEN; k++)
        m_macaddr[k] = macaddr[k];

    r_tx_chan_tdm_timer   = new sc_signal<uint32_t>[channels];
    r_tx_dispatch_packets = new sc_signal<uint32_t>[channels];

    r_tx_channel_to_channel = new sc_signal<uint32_t>*[channels];
    for ( size_t k = 0 ; k < channels ; k++)
        {
            r_tx_channel_to_channel[k] = new sc_signal<uint32_t>[channels];
        }

    for ( size_t k = 0 ; k < channels ; k++)
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

tmpl(/**/)::~VciMultiNic()
{
    for ( size_t k = 0 ; k < m_channels ; k++)
        {
            delete r_rx_channel[k];
            delete r_tx_channel[k];
            delete r_tx_channel_to_channel[k];
        }

    delete [] r_tx_channel_to_channel;
    delete [] r_tx_dispatch_packets;
    delete [] r_tx_chan_tdm_timer;

    delete r_channel_mac_4;
    delete r_channel_mac_2;

    delete r_rx_channel;
    delete r_tx_channel;

    close(r_backend_rx.get_fd()); // If the controller is used asynmetricly (output are gmii AND tap), then we should care more about that getter
}

}}


// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

