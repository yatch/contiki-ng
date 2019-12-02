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
 * \file
 *         MSF NumCells* counters APIs
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#include "contiki.h"
#include "lib/assert.h"

#include "services/shell/shell.h"

#include "msf.h"
#include "msf-autonomous-cell.h"
#include "msf-housekeeping.h"
#include "msf-negotiated-cell.h"
#include "msf-sixp.h"

#include "sys/log.h"
#define LOG_MODULE "MSF"
#define LOG_LEVEL LOG_LEVEL_MSF

static struct {
  uint16_t scheduled;
  uint16_t required;
  uint16_t elapsed;
  uint16_t used;
} tx_num_cells, rx_num_cells;
static bool need_keep_alive = false;

/*---------------------------------------------------------------------------*/
static
void update(msf_negotiated_cell_type_t cell_type)
{
  const linkaddr_t *parent_addr = msf_housekeeping_get_parent_addr();
  uint16_t max_num_cells_scheduled;
  uint16_t *num_cells_scheduled;
  uint16_t *num_cells_used;
  uint16_t *num_cells_required;
  uint16_t *num_cells_elapsed;
  uint16_t lim_num_cells_used_high;

  assert(parent_addr != NULL);

  /*
   * We're evaluating NumCellsUsed/NumCellsElapse, although we cannot
   * have a precise usage value as described in the MSF
   * draft. NumCellsUsed could be larger than NumCellsElapse. We may
   * overcount or undercount the number of elapsed negotiated
   * cells. In addition, tx_num_cells.used may have a count of the
   * autonomous TX cell.
   */
  if(cell_type == MSF_NEGOTIATED_CELL_TYPE_TX) {
    max_num_cells_scheduled = MSF_MAX_NUM_NEGOTIATED_TX_CELLS;
    num_cells_scheduled = &tx_num_cells.scheduled;
    num_cells_used = &tx_num_cells.used;
    num_cells_elapsed = &tx_num_cells.elapsed;
    num_cells_required = &tx_num_cells.required;
    lim_num_cells_used_high = MSF_LIM_NUM_CELLS_USED_HIGH;
  } else if(cell_type == MSF_NEGOTIATED_CELL_TYPE_RX) {
    max_num_cells_scheduled = MSF_MAX_NUM_NEGOTIATED_RX_CELLS;
    num_cells_scheduled = &rx_num_cells.scheduled;
    num_cells_used = &rx_num_cells.used;
    num_cells_elapsed = &rx_num_cells.elapsed;
    num_cells_required = &rx_num_cells.required;
    if(num_cells_scheduled > 0) {
      lim_num_cells_used_high = MSF_LIM_NUM_CELLS_USED_HIGH;
    } else {
      lim_num_cells_used_high = MSF_INITIAL_LIM_NUM_RX_CELLS_USED_HIGH;
    }
  } else {
    return;
  }

  /* updating the counters */
  *num_cells_scheduled = msf_negotiated_cell_get_num_cells(cell_type,
                                                           parent_addr);
  if(cell_type == MSF_NEGOTIATED_CELL_TYPE_RX &&
     *num_cells_scheduled == 0) {
    assert(msf_autonomous_cell_get_rx() != NULL);
    *num_cells_elapsed += 1;
  } else {
    *num_cells_elapsed += *num_cells_scheduled;
  }

  if(*num_cells_elapsed < MSF_MAX_NUM_CELLS) {
    LOG_DBG("for upward %s - NumCellsElapsed: %u, NumCellsUsed: %u, "
            "NumCellsScheduled: %u, NumCellsRequired: %u\n",
            cell_type == MSF_NEGOTIATED_CELL_TYPE_TX ? "TX" : "RX",
            *num_cells_elapsed, *num_cells_used,
            *num_cells_scheduled, *num_cells_required);
  } else {
    LOG_INFO("for upward %s - NumCellsElapsed: %u, NumCellsUsed: %u, "
             "NumCellsScheduled: %u, NumCellsRequired: %u\n",
             cell_type == MSF_NEGOTIATED_CELL_TYPE_TX ? "TX" : "RX",
             *num_cells_elapsed, *num_cells_used,
             *num_cells_scheduled, *num_cells_required);

    if(*num_cells_used > lim_num_cells_used_high &&
       *num_cells_scheduled < max_num_cells_scheduled &&
       *num_cells_required < (*num_cells_scheduled + 1)) {
      *num_cells_required = *num_cells_scheduled + 1;
      LOG_INFO("increment NumCellsRequired to %u; ", *num_cells_required);
      LOG_INFO_("going to add another negotiated %s cell\n",
                cell_type == MSF_NEGOTIATED_CELL_TYPE_TX ? "TX" : "RX");
    } else if(*num_cells_used < MSF_LIM_NUM_CELLS_USED_LOW &&
              ((cell_type == MSF_NEGOTIATED_CELL_TYPE_TX &&
                *num_cells_scheduled > 1) ||
               (cell_type == MSF_NEGOTIATED_CELL_TYPE_RX &&
                *num_cells_scheduled > 0)) &&
              *num_cells_required > (*num_cells_scheduled - 1)) {
      *num_cells_required = *num_cells_scheduled - 1;
      LOG_INFO("decrement NumCellsRequred to %u; ",
               *num_cells_required);
      LOG_INFO_("going to delete a negotiated %s cell\n",
                cell_type == MSF_NEGOTIATED_CELL_TYPE_TX ? "TX" : "RX");
    } else if(cell_type == MSF_NEGOTIATED_CELL_TYPE_TX &&
              *num_cells_used == 0 &&
              *num_cells_scheduled > 0){
      /*
       * Send a keep-alive message, which is a COUNT request, to
       * prevent the parent from removing negotiated cells scheduled
       * with us.
       */
      need_keep_alive = true;
    } else {
      /*
       *  We have a right amount of negotiated cells for the latest
       *  traffic
       */
    }
    /* reset NumCellsElapsed and NumCellsUsed */
    *num_cells_elapsed = 0;
    *num_cells_used = 0;
  }
}
/*---------------------------------------------------------------------------*/
void
msf_num_cells_reset(bool clear_num_cells_required)
{
  need_keep_alive = false;

  tx_num_cells.scheduled = 0;
  tx_num_cells.elapsed = 0;
  tx_num_cells.used = 0;

  rx_num_cells.scheduled = 0;
  rx_num_cells.elapsed = 0;
  rx_num_cells.used = 0;

  if(clear_num_cells_required) {
    tx_num_cells.required = 1;
    rx_num_cells.required = 0;
  } else {
    /* keep them */
  }
}
/*---------------------------------------------------------------------------*/
void
msf_num_cells_update(void)
{
  if(msf_housekeeping_get_parent_addr() == NULL) {
    /* nothing to do */
  } else {
    update(MSF_NEGOTIATED_CELL_TYPE_TX);
    update(MSF_NEGOTIATED_CELL_TYPE_RX);
  }
}
/*---------------------------------------------------------------------------*/
void
msf_num_cells_update_tx_used(uint16_t count)
{
  tx_num_cells.used += count;
}
/*---------------------------------------------------------------------------*/
void
msf_num_cells_increment_rx_used(void)
{
  rx_num_cells.used += 1;
}
/*---------------------------------------------------------------------------*/
void
msf_num_cells_trigger_6p_transaction(void)
{
  if(tx_num_cells.scheduled < tx_num_cells.required) {
    msf_sixp_add_send_request(MSF_NEGOTIATED_CELL_TYPE_TX);
  } else if(rx_num_cells.scheduled < rx_num_cells.required) {
    msf_sixp_add_send_request(MSF_NEGOTIATED_CELL_TYPE_RX);
  } else if(tx_num_cells.scheduled > tx_num_cells.required) {
    msf_sixp_delete_send_request(MSF_NEGOTIATED_CELL_TYPE_TX);
  } else if(rx_num_cells.scheduled > rx_num_cells.required) {
    msf_sixp_delete_send_request(MSF_NEGOTIATED_CELL_TYPE_RX);
  } else if(need_keep_alive) {
    msf_sixp_count_send_request();
    need_keep_alive = false;
  } else {
    /* nothing to do */
  }
}
/*---------------------------------------------------------------------------*/
void
msf_num_cells_show(shell_output_func output)
{
  SHELL_OUTPUT(output, "o up TX - NumCellsElapsed %u, NumCellsUsed: %u\n",
               tx_num_cells.elapsed, tx_num_cells.used);
  SHELL_OUTPUT(output, "o up TX - required: %u, scheduled: %u\n",
               tx_num_cells.required, tx_num_cells.scheduled);
  SHELL_OUTPUT(output, "o up RX - NumCellsElapsed N/A, NumCellsUsed: N/A\n");
  SHELL_OUTPUT(output, "o up RX - required: N/A, scheduled: N/A\n");
}
/*---------------------------------------------------------------------------*/
