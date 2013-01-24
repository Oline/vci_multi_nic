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
 * Copyright (c) UPMC, Lip6, Asim
 *         alain.greiner@lip6.fr
 *         Clement Devigne <clement.devigne@etu.upmc.fr>
 *         Sylvain Leroy <sylvain.leroy@lip6.fr>
 *
 * Maintainers: alain
 */

#ifndef SOCLIB_VCI_MULTI_NIC_H
#define SOCLIB_VCI_MULTI_NIC_H

#include <stdint.h>
#include <systemc>
#include "vci_target.h"
#include "caba_base_module.h"
#include "mapping_table.h"
#include "vci_multi_nic.h"
#include "fifo_multi_buffer.h"
#include "nic_rx_channel.h"
#include "nic_tx_channel.h"
#include "nic_rx_gmii.h"
#include "nic_tx_gmii.h"
#include "generic_fifo.h"

namespace soclib {
namespace caba {

using namespace sc_core;


template<typename vci_param>
class VciMultiNic
	: public caba::BaseModule
{
private:

    // Global CONFIGURATION and STATUS registers

#ifdef SOCLIB_PERF_NIC
    sc_signal<uint32_t>                     r_total_len_gmii;                   // register for count total Bytes receive by RX_GMII
    sc_signal<uint32_t>                     r_total_len_rx_chan;                // register for count total Bytes receive by RX_CHAN
    sc_signal<uint32_t>                     r_total_len_tx_chan;                // register for count total Bytes send    by TX_CHAN
    sc_signal<uint32_t>                     r_total_len_tx_gmii;                // register for count total Bytes send    by TX_GMII
#endif
    sc_signal<bool>                         r_broadcast_enable;                 // register broadcast mode enable when 1
    sc_signal<bool>                         r_nic_on;                           // register power enable when 1
    sc_signal<bool>                         r_rx_dispatch_broadcast;            // register set at 1 when broadcast detected

    // Channel CONFIGURATION and STATUS registers
    sc_signal<uint32_t>                     *r_channel_mac_4;                   // MAC address (first 4 bytes)
    sc_signal<uint32_t>                     *r_channel_mac_2;                   // MAC address (last 2 bytes)
    sc_signal<uint32_t>                     r_channel_active_channels;          // bitfield where bit N means : 0 -> channel N is disabled, 1 -> channel N is enabled
    sc_signal<uint32_t>                     r_channel_mac_addr_set;             // bitfield where bit N means : 0 -> channel N has NO MAC addr, 1 -> channel N has a MAC addr set
    sc_signal<uint32_t>                     *r_channel_;                        // MAC address extend
    sc_signal<uint32_t>                     r_rx_sel_channel_wok;               // bitfield where bit N means : 0 -> channel N is not WOK, 1 -> channel N is WOK
    sc_signal<uint32_t>                     r_rx_sel_space_timeout_ok;          // bitfield where bit N means : 0 -> channel N has not enough space or time for write, 1 -> channel N is good for write
    
    // VCI registers
    sc_signal<int>				            r_vci_fsm;
    sc_signal<typename vci_param::srcid_t>	r_vci_srcid;            // for rsrcid
    sc_signal<typename vci_param::trdid_t>	r_vci_trdid;            // for rtrdid
    sc_signal<typename vci_param::pktid_t>	r_vci_pktid;            // for rpktid
    sc_signal<typename vci_param::data_t>	r_vci_wdata;            // for write burst
    sc_signal<size_t>                       r_vci_channel;          // selected channel
    sc_signal<size_t>                       r_vci_ptw;              // write pointer
    sc_signal<size_t>                       r_vci_ptr;              // read pointer
    sc_signal<size_t>                       r_vci_nwords;           // word counter 
    sc_signal<uint32_t>                     r_vci_address;          // address vci (for TX_WRITE_BURST)

    // RX_G2S registers
    sc_signal<int>                          r_rx_g2s_fsm;
    sc_signal<uint32_t>                     r_rx_g2s_checksum;                // packet checksum
    sc_signal<uint8_t>                      r_rx_g2s_dt0;                     // local data buffer
    sc_signal<uint8_t>                      r_rx_g2s_dt1;                     // local data buffer
    sc_signal<uint8_t>                      r_rx_g2s_dt2;                     // local data buffer
    sc_signal<uint8_t>                      r_rx_g2s_dt3;                     // local data buffer
    sc_signal<uint8_t>                      r_rx_g2s_dt4;                     // local data buffer
    sc_signal<uint8_t>                      r_rx_g2s_dt5;                     // local data buffer
    sc_signal<size_t>                       r_rx_g2s_delay;                   // delay cycle (counter)
    sc_signal<uint32_t>                     r_rx_g2s_npkt;                    // packet receive counter (stat counter)
    sc_signal<uint32_t>                     r_rx_g2s_npkt_crc_success;        // packet receive checksum OK counter (stat counter)
    sc_signal<uint32_t>                     r_rx_g2s_npkt_crc_fail;           // packet receive checksum KO counter (stat counter)
    sc_signal<uint32_t>                     r_rx_g2s_npkt_err;                // packet receive ERR (stat counter)

    // RX_DES registers
    sc_signal<int>                          r_rx_des_fsm;
    sc_signal<uint32_t>                     r_rx_des_counter_bytes;            // nb bytes in one packet
    sc_signal<uint32_t>                     r_rx_des_padding;                  // padding
    sc_signal<uint8_t>*                     r_rx_des_data;                     // array[4]
    sc_signal<uint32_t>                     r_rx_des_npkt_err_in_des;          // packet receive in error because of plen not valid or multi_fifo is full (stat counter)
    sc_signal<uint32_t>                     r_rx_des_npkt_write_mfifo_success; // packet receive write success in mfifo (stat counter)
    sc_signal<uint32_t>                     r_rx_des_npkt_small;               // packet receive err cause of plen < 64 B (stat counter)
    sc_signal<uint32_t>                     r_rx_des_npkt_overflow;            // packet receive err cause of plen > 1518 B (stat counter)
    sc_signal<uint32_t>                     r_rx_des_npkt_err_mfifo_full;      // packet receive err cause of mfifo full (stat counter)

    // RX_DISPATCH registers
    sc_signal<int>                          r_rx_dispatch_fsm;
    sc_signal<uint32_t>                     r_rx_dispatch_channel;               // channel index
    sc_signal<bool>                         r_rx_dispatch_bp;                    // fifo index
    sc_signal<uint32_t>                     r_rx_dispatch_plen;                  // packet length (bytes)
    sc_signal<uint32_t>                     r_rx_dispatch_dt0;                   // word value
    sc_signal<uint32_t>                     r_rx_dispatch_data;                  // word value
    sc_signal<uint32_t>                     r_rx_dispatch_words;                 // write words (counter)
    sc_signal<uint32_t>                     r_rx_dispatch_npkt_skip_adrmac_fail; // packet receive skip because of mac addr was wrong/not found for any channel (stat counter)
    sc_signal<uint32_t>                     r_rx_dispatch_npkt_wchannel_success; // packet receive write in channel success (stat counter)
    sc_signal<uint32_t>                     r_rx_dispatch_npkt_wchannel_fail;    // packet receive write in channel fail because channel was full (stat counter)

    // TX_DISPATCH registers
    sc_signal<int>                          r_tx_dispatch_fsm;
    sc_signal<size_t>                       r_tx_dispatch_channel;               // channel index
    sc_signal<uint32_t>                     r_tx_dispatch_data;                  // word value
    sc_signal<uint32_t>                     *r_tx_dispatch_packets;              // number of packets for each channels
    sc_signal<uint32_t>                     r_tx_dispatch_words;                 // read words counter
    sc_signal<uint32_t>                     r_tx_dispatch_bytes;                 // bytes in last word
    sc_signal<bool>                         r_tx_dispatch_first_bytes_pckt;      // Bool for detect the first Byte in a packet
    sc_signal<uint32_t>                     r_tx_npkt;                           // packet send (stat counter)
    sc_signal<uint32_t>                     r_tx_npkt_overflow;                  // packet skip because of overflow length >1518B (stat counter)
    sc_signal<uint32_t>                     r_tx_npkt_small;                     // packet skip because of too small length <64B (stat counter)
    sc_signal<uint32_t>                     r_tx_dispatch_addr_mac_src_fail;     // number of packet skip because of address mac src bad (stat counter)
    sc_signal<uint32_t>                     r_tx_dispatch_dt0;                   // Step 1 of tx_dispatch pipeline
    sc_signal<uint32_t>                     r_tx_dispatch_dt1;                   // Step 2 of tx_dispatch pipeline
    sc_signal<uint32_t>                     r_tx_dispatch_dt2;                   // Step 3 of tx_dispatch pipeline
    sc_signal<uint32_t>                     r_tx_dispatch_interne;               // register for detect a transmit interne
    sc_signal<uint32_t>                     r_tx_dispatch_pipe_count;            // counter for empty the tx_dispatch pipeline
    sc_signal<bool>                         r_tx_dispatch_broadcast;             // register for detect a transmit broadcast
    sc_signal<uint32_t>                     r_tx_dispatch_channel_interne_send;  // register used for set the target of transmit interne
    sc_signal<uint32_t>                     r_tx_dispatch_ifg;                   // Counter for TX_IFG

    // TX_S2G registers
    sc_signal<int>                          r_tx_s2g_fsm;
    sc_signal<uint32_t>                     r_tx_s2g_checksum;                   // packet checksum
    sc_signal<uint8_t>                      r_tx_s2g_data;                       // local data buffer
    sc_signal<size_t>                       r_tx_s2g_index;                      // checksum byte index

    sc_signal<uint32_t>                     r_tx_tdm_enable;                     // register for enable tdm mode
    sc_signal<uint32_t>                     r_tx_tdm_timer;                      // register timer for tdm
    sc_signal<uint32_t>                     r_tx_chan_sel_tdm;                   // register for select channel/token of tdm mode 
    sc_signal<uint32_t>                     *r_tx_chan_tdm_timer;                // registers for init value of each tdm time of channels

    // channels
    NicRxChannel**                          r_rx_channel;                        // array[m_channels]
    NicTxChannel**                          r_tx_channel;                        // array[m_channels]

    sc_signal<uint32_t>**                    r_tx_channel_to_channel;            // Counters of transmit interne (array[m_channels][m_channels])

    // fifos
    GenericFifo<uint16_t>                   r_rx_fifo_stream;
    FifoMultiBuffer                         r_rx_fifo_multi;
    GenericFifo<uint16_t>                   r_tx_fifo_stream;
    FifoMultiBuffer                         r_bp_fifo_multi;

    // Packet in and out
    NicRxGmii                               r_gmii_rx;
    NicTxGmii                               r_gmii_tx;

    // sructural parameters
    soclib::common::Segment			        m_segment;
    const size_t				            m_channels;		// no more than 8

    // methods
    void        transition();
    void        genMoore();
    uint32_t    read_register(uint32_t addr); 

protected:

    SC_HAS_PROCESS(VciMultiNic);

public:

    // FSM states
    enum vci_tgt_fsm_state_e {
        VCI_IDLE,
        VCI_WRITE_TX_BURST,
        VCI_READ_RX_BURST,
        VCI_WRITE_TX_LAST,
        VCI_WRITE_REG,
        VCI_READ_REG,
        VCI_ERROR,
    };
    enum rx_g2s_fsm_state_e {
        RX_G2S_IDLE,
        RX_G2S_DELAY,
        RX_G2S_LOAD,
        RX_G2S_SOS,
        RX_G2S_LOOP,
        RX_G2S_END,
        RX_G2S_EXTD,
        RX_G2S_ERR,
        RX_G2S_FAIL,
    };
    enum rx_des_fsm_state_e {
    	RX_DES_READ_0,
	    RX_DES_READ_1,
	    RX_DES_READ_2,
	    RX_DES_READ_3,
	    RX_DES_READ_WRITE_0,
	    RX_DES_READ_WRITE_1,
	    RX_DES_READ_WRITE_2,
	    RX_DES_READ_WRITE_3,
	    RX_DES_WRITE_LAST,
	    RX_DES_WRITE_CLEAR,
    };
    enum rx_dispatch_fsm_state_e {
        RX_DISPATCH_IDLE,
        RX_DISPATCH_GET_PLEN,
        RX_DISPATCH_READ_FIRST,
        RX_DISPATCH_CHANNEL_SELECT,
        RX_DISPATCH_PACKET_SKIP,
        RX_DISPATCH_GET_WOK,
        RX_DISPATCH_GET_CHANNEL_BROADCAST,
        RX_DISPATCH_CLOSE_CONT,
        RX_DISPATCH_GET_SPACE,
        RX_DISPATCH_CHECK_MAC_SRC,
        RX_DISPATCH_READ_WRITE,
        RX_DISPATCH_WRITE_LAST,
    };
    enum tx_dispatch_fsm_state_e {
        TX_DISPATCH_IDLE,
        TX_DISPATCH_GET_NPKT,
        TX_DISPATCH_GET_PLEN,
        TX_DISPATCH_SKIP_PKT, 
        TX_DISPATCH_READ_FIRST,
        TX_DISPATCH_FIFO_SELECT,
        TX_DISPATCH_CHECK_MAC_ADDR_SRC,
        TX_DISPATCH_READ_WRITE_BP,
        TX_DISPATCH_WRITE_LAST_BP,
        TX_DISPATCH_WRITE_B0_TX,
        TX_DISPATCH_WRITE_B1_TX,
        TX_DISPATCH_WRITE_B2_TX,
        TX_DISPATCH_READ_WRITE_TX,
        TX_DISPATCH_IFG,
        TX_DISPATCH_RELEASE_CONT,
    };
    enum tx_s2g_fsm_state_e {
        TX_S2G_IDLE,
        TX_S2G_WRITE_DATA,
        TX_S2G_WRITE_LAST_DATA,
        TX_S2G_WRITE_CS,
    };

    // Stream types
    enum stream_type_e {
        STREAM_TYPE_SOS,     // start of stream
        STREAM_TYPE_EOS,     // end of stream
        STREAM_TYPE_ERR,     // corrupted end of stream
        STREAM_TYPE_NEV,     // no special event
    };

    // ports
    sc_in<bool> 				            p_clk;
    sc_in<bool> 				            p_resetn;
    soclib::caba::VciTarget<vci_param> 		p_vci;
    sc_out<bool>* 				            p_rx_irq;
    sc_out<bool>* 				            p_tx_irq;

#ifdef SOCLIB_PERF_NIC
    // return total number Bytes count in rx_gmii
    inline uint32_t get_total_len_gmii()
    {
        return r_total_len_gmii.read();
    }

    // return total number Bytes successfull write in rx_channel
    inline uint32_t get_total_len_rx_chan()
    {
        return r_total_len_rx_chan.read();
    }
    
    // return total number Bytes read from a tx_channel
    inline uint32_t get_total_len_tx_chan()
    {
        return r_total_len_tx_chan.read();
    }

    // return total number Bytes write in the output_file
    inline uint32_t get_total_len_tx_gmii()
    {
        return r_total_len_tx_gmii.read();
    }

    // return total number of packets successful to enter mfifo
    inline uint32_t get_rx_des_npkt_write_mfifo_success()
    {
        return r_rx_des_npkt_write_mfifo_success.read();
    }

    // return total number of packets failing in DES
    inline uint32_t get_rx_des_npkt_err_in_des()
    {
        return r_rx_des_npkt_err_in_des.read();
    }

    // return total number of packets with bad MAC ADDR
    inline uint32_t get_rx_dispatch_npkt_skip_adrmac_fail()
    {
        return r_rx_dispatch_npkt_skip_adrmac_fail.read();
    }

    // return total number of packets not writen in a channel because of channel full
    inline uint32_t get_rx_dispatch_npkt_wchannel_fail()
    {
        return r_rx_dispatch_npkt_wchannel_fail.read();
    }

    // return total number of packets writen in a channel
    inline uint32_t get_rx_dispatch_npkt_wchannel_success()
    {
        return r_rx_dispatch_npkt_wchannel_success.read();
    }

#endif

    // Public functions
    void print_trace(uint32_t option = 0);

    // Contructor
    VciMultiNic( sc_module_name 			name,
                 const soclib::common::IntTab 		&tgtid,
                 const soclib::common::MappingTable 	&mt,
                 const size_t				        channels,           // number of channels
                 const char*                         rx_file_pathname,   // received packets
                 const char*                         tx_file_pathname,   // transmitted packets
                 const size_t                        timeout);           // max waiting cycles
};

}}

#endif

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

