/*
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
 *         Alain Greiner <alain.greiner@lip6.fr>, 2012
 *         Clement Devigne <clement.devigne@etu.upmc.fr>
 *         Sylvain Leroy <sylvain.leroy@lip6.fr>
 *
 * Maintainers: alain
 */

#ifndef MULTI_NIC_REGS_H
#define MULTI_NIC_REGS_H

#define NB_NIC_CHANNELS 8

enum SoclibMultiNicHyperviseurConfigurationRegisters {
    // CHANNEL MANAGEMENT
    VIS,                         // bitfield where bit N means : 0 -> channel N is disabled, 1 -> channel N is enabled
    GENERAL_CHAN_MAC_ADDR_SET,   // bitfield where bit N means : 0 -> channel N has NO MAC addr, 1 -> channel N has a MAC addr set
    // CONTROLER MANAGEMENT
    GENERAL_MAC_4,
    GENERAL_MAC_2,
    NIC_ON,                      // NIC Power Enable
    BROADCAST_ENABLE,            // Enable Broadcast if 1
    TDM_ENABLE,                  // Enable TDM Scheduler in TX if 1, or RR Scheduler if 0
    TDM_TIMERS,                  // Set initial value of TMD Timers for each channels (array[m_channels])
};

enum SoclibMultiNicHyperviseurStatusRegisters {
    // RX_G2S
    RX_PKT,                     // Count every packets going through stage 1
    RX_CRC_SUCCESS,             // Successful CRC packets
    RX_CRC_FAIL,                // Unsuccessful CRC packets
    RX_ERR_MII,                 // Error in receiving from (G)MII
    // RX_DES
    RX_MFIFO_SUCCESS,           // Count every packets going through stage 2
    RX_ERR_SMALL,               // Count every too small packet (< 64B)
    RX_ERR_OVERFLOW,            // Count every too small packet (> 1518B)
    RX_ERR_MFIFO_FULL,          // Count every packets dropped because of stage 2 full
    RX_ERR_IN_DES,              // Count every packet received in error (RX_ERR_SMALL + RX_ERR_OVERFLOW + RX_ERR_MFIFO_FULL)
    // RX_DISPATCH
    RX_CHANNEL_SUCCESS,         // Count every packets going through stage 3 (final stage)
    RX_CHANNEL_FAIL,            // Count every packets dropped because of no room in the containers
    RX_MAC_ADDR_FAIL,           // Count every packets dropped because of mac addr was wrong/not found for any channel
    // TX_DISPATCH
    TX_PKT,                     // Count every packets in transmit
    TX_ERR_SMALL,               // Count every packets too small in transmit (<60B)
    TX_ERR_OVERFLOW,            // Count every packets too long in transmit  (>1514B)
    TX_REG,                     // Count every packets transmit in internal mode (array[m_channels][m_channels])
};

enum SoclibMultiNicChannelsConfigurationRegisters {
    RX_RELEASE,                 // Release a RX_CONTAINER after Read
    TX_CLOSE,                   // Close a TX_CONTAINER after Write
    TIMEOUT,                    // Set a new value for TIMEOUT RX_CONTAINER
};

enum SoclibMultiNicChannelsStatusRegisters {
    RX_ROK,                     // Register used to known if a RX_CONTAINER is ready to be read
    TX_WOK,                     // Register used to known if a TX_CONTAINER is ready to be write
    MAC_4,                      // Read : return value of MAC_4 for channel[i]
                                // Write (hypervisor) : set a new MAC_4 for channel[i]
    MAC_2,                      // Read : return value of MAC_2 for channel[i]
                                // Write (hypervisor) : set a new MAC_2 for channel[i]
    RX_NWORDS,                  // Return number of useful words in RX_CONTAINER
};

#endif

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

