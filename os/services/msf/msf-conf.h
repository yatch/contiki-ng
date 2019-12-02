/*
 * Copyright (c) 2019, Inria.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \addtogroup msf
 * @{
 */
/**
 * \file
 *         MSF configuration parameters
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#ifndef _MSF_CONF_H_
#define _MSF_CONF_H_

/**
 * \brief The length of slotframes for MSF; should be the same as
 * slotframe 0
 */
#ifdef MSF_CONF_SLOTFRAME_LENGTH
#define MSF_SLOTFRAME_LENGTH MSF_CONF_SLOTFRAME_LENGTH
#else
#define MSF_SLOTFRAME_LENGTH TSCH_SCHEDULE_DEFAULT_LENGTH
#endif

/**
 * \brief The slot length spedifiend in milliseconds
 */
#ifdef MSF_CONF_SLOT_LENGTH_MS
#define MSF_SLOT_LENGTH_MS MSF_CONF_SLOT_LENGTH_MS
/*
 * We specify a decimal value instead of using
 * TSCH_DEFAULT_TIMESLOT_TIMING[tsch_ts_timeslot_length] to define
 * MSF_SLOT_LENGTH_MS since referring TSCH_DEFAULT_TIMESLOT_TIMING is
 * not allowed to define a constant value, const sixtop_sf_t msf.
 */
#else
#define MSF_SLOT_LENGTH_MS 10
#endif /* MSF_CONF_SLOT_LENGTH_MS */

/**
 * \brief The maximum number of negotiated TX cells to a parent
 */
#ifdef MSF_CONF_MAX_NUM_NEGOTIATED_TX_CELLS
#define MSF_MAX_NUM_NEGOTIATED_TX_CELLS MSF_CONF_MAX_NUM_NEGOTIATED_TX_CELLS
#else
#define MSF_MAX_NUM_NEGOTIATED_TX_CELLS 8
#endif /* MSF_CONF_MAX_NUM_OF_NEGOTIATED_TX_CELLS */

/**
 * \brief The maximum number of negotiated RX cells to a parent
 */
#ifdef MSF_CONF_MAX_NUM_NEGOTIATED_RX_CELLS
#define MSF_MAX_NUM_NEGOTIATED_RX_CELLS MSF_CONF_MAX_NUM_NEGOTIATED_RX_CELLS
#else
#define MSF_MAX_NUM_NEGOTIATED_RX_CELLS 8
#endif /* MSF_CONF_MAX_NUM_OF_NEGOTIATED_TX_CELLS */

/**
 * \brief The value of NumCellsElapse to update NumCellsRequired
 */
#ifdef MSF_CONF_MAX_NUM_CELLS
#define MSF_MAX_NUM_CELLS MSF_CONF_MAX_NUM_CELLS
#else
#define MSF_MAX_NUM_CELLS 100
#endif /* MSF_CONF_MAX_NUM_CELLS */

/**
 * \brief The minimum value of NumCellsUsed to add a negotiated cell
 */
#ifdef MSF_CONF_LIM_NUM_CELLS_USED_HIGH
#define MSF_LIM_NUM_CELLS_USED_HIGH MSF_CONF_LIM_NUM_CELLS_USED_HIGH
#else
#define MSF_LIM_NUM_CELLS_USED_HIGH 75
#endif /* MSF_CONF_LIM_NUM_CELLS_USED_HIGH */

/**
 * \brief The initial minimum value of NumCellsUsed to add a *RX*
 * negotiated cell.
 *
 * \details This value is used only while there is no negotiated RX
 * cell scheduled. This value is expected to have a smaller value than
 * MSF_CONF_LIM_NUM_CELLS_USED_HIGH to take into account the fact that
 * an autonomous RX cell is a shared cell, where the retransmission
 * backoff is performed.
 */
#ifdef MSF_CONF_INITIAL_LIM_NUM_RX_CELLS_USED_HIGH
#define MSF_INITIAL_LIM_NUM_RX_CELLS_USED_HIGH \
  MSF_CONF_INITIAL_LIM_NUM_RX_CELLS_USED_HIGH
#else
#define MSF_INITIAL_LIM_NUM_RX_CELLS_USED_HIGH 50
#endif /* MSF_CONF_LIM_NUM_CELLS_USED_HIGH */

/**
 * \brief The maximum value of NumCellsUsed to delete a negotiated cell
 */
#ifdef MSF_CONF_LIM_NUM_CELLS_USED_LOW
#define MSF_LIM_NUM_CELLS_USED_LOW MSF_CONF_LIM_NUM_CELLS_USED_LOW
#else
#define MSF_LIM_NUM_CELLS_USED_LOW 25
#endif /* MSF_CONF_LIM_NUM_CELLS_USED_LOW */

/**
 * \brief The interval to evaluate PDR for each negotiated TX cell
 */
#ifdef MSF_CONF_HOUSEKEEPING_COLLISION_PERIOD_MIN
#define MSF_HOUSEKEEPING_COLLISION_PERIOD_MIN   \
  MSF_CONF_HOUSEKEEPING_COLLISION_PERIOD_MIN
#else
#define MSF_HOUSEKEEPING_COLLISION_PERIOD_MIN 1
#endif /* MSF_CONF_HOUSEKEEPING_COLLISION_PERIOD_MIN */

/**
 * \brief The interval of garbage collection
 */
#ifdef MSF_CONF_HOUSEKEEPING_GC_PERIOD_MIN
#define MSF_HOUSEKEEPING_GC_PERIOD_MIN \
  MSF_CONF_HOUSEKEEPING_GC_PERIOD_MIN
#else
#define MSF_HOUSEKEEPING_GC_PERIOD_MIN 2
#endif /* MSF_CONF_HOUSEKEEPING_GC_PERIOD_MIN */

/**
 * \brief The minimum number of NumCellsTx to calculate PDR
 */
#ifdef MSF_CONF_MIN_NUM_TX_FOR_RELOCATION
#define MSF_MIN_NUM_TX_FOR_RELOCATION MSF_CONF_MIN_NUM_TX_FOR_RELOCATION
#else
#define MSF_MIN_NUM_TX_FOR_RELOCATION 128
#endif /* MSF_CONF_MIN_NUM_TX_FOR_RELOCATION */

/**
 * \brief The minimum PDR gap to determine relocation
 */
#ifdef MSF_CONF_RELOCATE_PDR_THRESHOLD
#define MSF_RELOCATE_PDR_THRESHOLD MSF_CONF_RELOCATE_PDR_THRESHOLD
#else
#define MSF_RELOCATE_PDR_THRESHOLD 50
#endif /* MSF_CONF_RELOCATE_PDR_THRESHOLD */

/**
 * \brief The minimum wait time for a next 6P request
 */
#ifdef MSF_CONF_WAIT_DURATION_MIN_SECONDS
#define MSF_WAIT_DURATION_MIN_SECONDS MSF_CONF_WAIT_DURATION_MIN_SECONDS
#else
#define MSF_WAIT_DURATION_MIN_SECONDS 30
#endif /* MSF_CONF_WAIT_DURATION_MIN_SECONDS */

/**
 * \brief The maximum wait time for a next 6P request
 */
#ifdef MSF_CONF_WAIT_DURATION_MAX_SECONDS
#define MSF_WAIT_DURATION_MAX_SECONDS MSF_CONF_WAIT_DURATION_MAX_SECONDS
#else
#define MSF_WAIT_DURATION_MAX_SECONDS 60
#endif /* MSF_CONF_WAIT_DURATION_MAX_SECONDS */

#endif /* !_MSF_CONF_H_ */

/** @} */
