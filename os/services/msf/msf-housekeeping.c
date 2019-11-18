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
 *         MSF Housekeeping
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#include <stdbool.h>

#include "contiki.h"
#include "lib/assert.h"

#include "net/linkaddr.h"
#include "net/mac/tsch/sixtop/sixtop.h"
#include "net/mac/tsch/sixtop/sixp-trans.h"
#include "services/shell/shell.h"

#include "msf.h"
#include "msf-negotiated-cell.h"
#include "msf-autonomous-cell.h"
#include "msf-sixp.h"

#include "sys/log.h"
#define LOG_MODULE "MSF"
#define LOG_LEVEL LOG_LEVEL_MSF

/* variables */
static linkaddr_t parent_addr_storage;
static linkaddr_t *parent_addr;
static uint16_t num_required_upward_cells;
static uint16_t num_cells_elapse;
static uint16_t num_cells_used;
static tsch_link_t *cell_to_relocate;

PROCESS(msf_housekeeping_process, "MSF housekeeping");

/* static functions */
static void update_num_required_upward_cells(void);
static void exec_postponed_cell_deletion(tsch_slotframe_t *slotframe);

/*---------------------------------------------------------------------------*/
static void
update_num_required_upward_cells(void)
{
  unsigned int num_negotiated_tx_cells;
  num_negotiated_tx_cells = msf_negotiated_cell_get_num_tx_cells(parent_addr);

  /*
   * We're evaluating NumCellsUsed/NumCellsElapse, although we cannot
   * have a precise usage value as described in the MSF
   * draft. NumCellsUsed could be larger than NumCellsElapse. We may
   * overcount or undercount the number of elapsed negotiated
   * cells. In addition, num_cells_used may have a count of the
   * autonomous TX cell.
   */
  LOG_INFO("NumCellsElapsed: %u, NumCellsUsed: %u,"
           "NumRequiredUpwardcells: %u\n",
           num_cells_elapse, num_cells_used, num_required_upward_cells);

  if(num_cells_used > MSF_LIM_NUM_CELLS_USED_HIGH &&
     num_negotiated_tx_cells < MSF_MAX_NUM_NEGOTIATED_TX_CELLS &&
     num_required_upward_cells < (num_negotiated_tx_cells + 1)) {
    num_required_upward_cells = num_negotiated_tx_cells + 1;
    LOG_INFO("increment NumRequiredUpwardCells to %u; ",
             num_required_upward_cells);
    LOG_INFO_("going to add another negotiated TX cell\n");
  } else if(num_cells_used < MSF_LIM_NUM_CELLS_USED_LOW &&
            num_negotiated_tx_cells > 1 &&
            num_required_upward_cells > (num_negotiated_tx_cells - 1)) {
    num_required_upward_cells = num_negotiated_tx_cells - 1;
    LOG_INFO("decrement NumRequiredUpwardCells to %u; ",
             num_required_upward_cells);
    LOG_INFO_("going to delete a negotiated TX cell\n");
  } else {
    /*
     *  We have a right amount of negotiated cells for the latest
     *  traffic
     */
  }
}
/*---------------------------------------------------------------------------*/
static void
exec_postponed_cell_deletion(tsch_slotframe_t *slotframe)
{
  if(slotframe == NULL) {
    /* do nothing */
  } else {
    tsch_link_t *cell, *next_cell;
    for(cell = list_head(slotframe->links_list);
        cell != NULL;
        cell = next_cell) {
      next_cell = list_item_next(cell);
      if(cell->link_options == LINK_OPTION_LINK_TO_DELETE) {
        LOG_DBG("delete a cell at slot_offset: %u, channel_offset: %u\n",
                cell->timeslot, cell->channel_offset);
        (void)tsch_schedule_remove_link(slotframe, cell);
      } else {
        /* need to keep this cell */
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(msf_housekeeping_process, ev, data)
{
  static struct etimer et;
  static struct timer t;
  const clock_time_t slotframe_interval = (CLOCK_SECOND /
                                           TSCH_SLOTS_PER_SECOND *
                                           MSF_SLOTFRAME_LENGTH);
  unsigned int num_negotiated_tx_cells;

  PROCESS_BEGIN();
  etimer_set(&et, slotframe_interval);
  timer_set(&t, MSF_HOUSEKEEPING_COLLISION_PERIOD_MIN * 60 * CLOCK_SECOND);

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_POLL ||
                             etimer_expired(&et));
    etimer_reset(&et);

    if(ev == PROCESS_EVENT_POLL) {
      exec_postponed_cell_deletion(msf_autonomous_cell_get_slotframe());
      exec_postponed_cell_deletion(msf_negotiated_cell_get_slotframe());
      if(msf_is_activated()) {
        continue;
      } else {
        /* stopping this process */
        break;
      }
    } else { /* etimer_expired(&et)  */
      if(parent_addr == NULL) {
        continue;
      } else {
        /* go through */
      }
    }

    /* update the counters */
    num_negotiated_tx_cells = msf_negotiated_cell_get_num_tx_cells(parent_addr);
    num_cells_elapse += num_negotiated_tx_cells;

    if(num_cells_elapse >= MSF_MAX_NUM_CELLS) {
      update_num_required_upward_cells();
      /* reset the counters */
      num_cells_elapse = 0;
      num_cells_used = 0;
    }

    /* decide to relocate a cell or not */
    if(timer_expired(&t) && cell_to_relocate == NULL) {
      cell_to_relocate = msf_negotiated_cell_get_cell_to_relocate();
      timer_restart(&t);
    }

    /* start an ADD or a DELETE transaction if necessary and possible */
    if(sixp_trans_find(parent_addr) == NULL &&
       msf_sixp_is_request_wait_timer_expired()) {
      if(num_negotiated_tx_cells < num_required_upward_cells) {
        msf_sixp_add_send_request();
      } else if(num_negotiated_tx_cells > num_required_upward_cells) {
        msf_sixp_delete_send_request();
      } else if(cell_to_relocate != NULL) {
        msf_sixp_relocate_send_request(cell_to_relocate);
      } else {
        /* nothing to do */
      }
    } else {
      /*
       * We cannot send a request since we're busy on an on-going
       * transaction with the parent. try it later.
       */
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
msf_housekeeping_start(void)
{
  parent_addr = NULL;
  num_required_upward_cells = 1;
  num_cells_elapse = 0;
  num_cells_used = 0;
  cell_to_relocate = NULL;
  process_start(&msf_housekeeping_process, NULL);
}
/*---------------------------------------------------------------------------*/
void
msf_housekeeping_stop(void)
{
  if(process_is_running(&msf_housekeeping_process)) {
    process_poll(&msf_housekeeping_process);
  } else {
    /* the houskeeping process is not running; do nothing */
  }
}
/*---------------------------------------------------------------------------*/
void
msf_housekeeping_set_parent_addr(const linkaddr_t *new_parent)
{
  assert(msf_is_activated() == true);

  LOG_INFO("switch parent from ");
  LOG_INFO_LLADDR(parent_addr);
  LOG_INFO_(" to ");
  LOG_INFO_LLADDR(new_parent);
  LOG_INFO_("\n");

  if(parent_addr != NULL) {
    /* CLEAR all the cells scheduled with the old parent */
    sixp_trans_t *trans = sixp_trans_find(parent_addr);
    if(trans != NULL) {
      /* abort the ongoing transaction */
      sixp_trans_abort(trans);
    }

    if(msf_negotiated_cell_get_num_tx_cells(parent_addr) > 0) {
      msf_sixp_clear_send_request(parent_addr);
    }
  }

  /* reset the counters */
  num_cells_elapse = 0;
  num_cells_used = 0;
  cell_to_relocate = NULL;
  /* start allocating negotiated cells with new_parent */
  /* reset the timer so as to send a request immediately */
  msf_sixp_stop_request_wait_timer();
  assert(msf_sixp_is_request_wait_timer_expired());

  if(new_parent == NULL) {
    parent_addr = NULL;
    /* reset the counter of required negotiated TX cells */
    LOG_DBG("resetting num_required_upward_cells to 1\n");
    num_required_upward_cells = 1;
  } else {
    parent_addr = &parent_addr_storage;
    linkaddr_copy(parent_addr, new_parent);
    /* keep num_required_upward_cells and use it for the new parent */
    LOG_DBG("we're going to schedule %u negotiated TX cell%s",
            num_required_upward_cells,
            num_required_upward_cells == 1 ? " " : "s ");
    LOG_DBG_("with the new parent\n");
    if(sixp_trans_find(parent_addr) == NULL) {
      msf_sixp_add_send_request();
    } else {
      /* we may have a transaction with the new parent */
      msf_sixp_start_request_wait_timer();
    }
  }
}
/*---------------------------------------------------------------------------*/
const linkaddr_t *
msf_housekeeping_get_parent_addr(void)
{
  return (const linkaddr_t *)parent_addr;
}
/*---------------------------------------------------------------------------*/
void
msf_housekeeping_update_num_cells_used(uint16_t count)
{
  num_cells_used += count;
}
/*---------------------------------------------------------------------------*/
int
msf_housekeeping_delete_cell_to_relocate(void)
{
  int ret;
  if(cell_to_relocate == NULL) {
    /* do nothing */
    ret = 0;
  } else {
    msf_negotiated_cell_delete(cell_to_relocate);
    cell_to_relocate = NULL;
  }
  return ret;
}
/*---------------------------------------------------------------------------*/
void
msf_housekeeping_resolve_inconsistency(const linkaddr_t *peer_addr)
{
  sixp_trans_abort(sixp_trans_find(peer_addr));
  msf_sixp_clear_send_request(peer_addr);
}
/*---------------------------------------------------------------------------*/
void
msf_housekeeping_delete_cell_later(tsch_link_t *cell)
{
  if(cell == NULL) {
    /* do nothing */
  } else {
    cell->link_options = LINK_OPTION_LINK_TO_DELETE;
    process_poll(&msf_housekeeping_process);
  }
}
/*---------------------------------------------------------------------------*/
void
msf_housekeeping_show_negotiated_cells(shell_output_func output)
{
  tsch_slotframe_t *slotframe = msf_negotiated_cell_get_slotframe();
  if(output == NULL) {
    /* do nothing */
  } else {
    SHELL_OUTPUT(output, "MSF Negotiated Cells:");
    if(slotframe == NULL || list_head(slotframe->links_list) == NULL) {
      SHELL_OUTPUT(output, " (none)\n");
    } else {
      SHELL_OUTPUT(output, "\n type, sl_off, ch_off,  PDR, addr\n");
      for(tsch_link_t *cell = list_head(slotframe->links_list);
          cell != NULL;
          cell = list_item_next(cell)) {
        SHELL_OUTPUT(output, "o ");
        SHELL_OUTPUT(output, " %s,    %3u,    %3u, ",
                     cell->link_options & LINK_OPTION_TX ? "TX" : "RX",
                     cell->timeslot,
                     cell->channel_offset);
        if(cell->link_options & LINK_OPTION_TX) {
          SHELL_OUTPUT(output, "%3u%%, ",
                       msf_negotiated_cell_get_num_tx_ack(cell)* 100 /
                       msf_negotiated_cell_get_num_tx(cell));
        } else {
          SHELL_OUTPUT(output, ", N/A, ");
        }
        shell_output_lladdr(output, &cell->addr);
        SHELL_OUTPUT(output, "\n");
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
void
msf_housekeeping_show_autonomous_cells(shell_output_func output)
{
  tsch_slotframe_t *slotframe = msf_autonomous_cell_get_slotframe();

  if(output == NULL) {
    /* do nothig */
  } else {
    SHELL_OUTPUT(output, "MSF Autonomous Cells:");
    if(slotframe == NULL || list_head(slotframe->links_list) == NULL) {
      SHELL_OUTPUT(output, " (none)\n");
    } else {
      SHELL_OUTPUT(output, "\n type, slot, ch_off, addr\n");
      for(tsch_link_t *cell = list_head(slotframe->links_list);
          cell != NULL;
          cell = list_item_next(cell)) {
        SHELL_OUTPUT(output, "o ");
        SHELL_OUTPUT(output, " %s,  %3u,   %3u, ",
                     cell->link_options & LINK_OPTION_TX ? "TX" : "RX",
                     cell->timeslot,
                     cell->channel_offset);
        shell_output_lladdr(output, &cell->addr);
        SHELL_OUTPUT(output, "\n");
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
uint16_t
msf_housekeeping_get_required_tx_cells(void)
{
  return num_required_upward_cells;
}
/*---------------------------------------------------------------------------*/
uint16_t
msf_housekeeping_get_num_tx_cells_elapsed(void)
{
  return num_cells_elapse;
}
/*---------------------------------------------------------------------------*/
uint16_t
msf_housekeeping_get_num_tx_cells_used(void)
{
  return num_cells_used;
}
/*---------------------------------------------------------------------------*/
