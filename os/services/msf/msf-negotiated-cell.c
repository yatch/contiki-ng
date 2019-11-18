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
 *         MSF Negotiated Cell APIs
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#include "contiki.h"
#include "lib/assert.h"
#include "lib/list.h"
#include "lib/memb.h"
#include "lib/random.h"

#include "net/mac/tsch/tsch.h"

#include "msf.h"
#include "msf-autonomous-cell.h"
#include "msf-housekeeping.h"
#include "msf-negotiated-cell.h"

#include "sys/log.h"
#define LOG_MODULE "MSF"
#define LOG_LEVEL LOG_LEVEL_MSF

typedef struct {
  tsch_link_t *next;
  uint16_t num_tx;
  uint16_t num_tx_ack;
} msf_negotiated_cell_data_t;

/* variables */
static tsch_slotframe_t *slotframe;

MEMB(msf_negotiated_cell_data_memb,
     msf_negotiated_cell_data_t,
     MSF_MAX_NUM_NEGOTIATED_TX_CELLS);

/*---------------------------------------------------------------------------*/
void
msf_negotiated_cell_activate(void)
{
  memb_init(&msf_negotiated_cell_data_memb);
  slotframe = tsch_schedule_get_slotframe_by_handle(
    MSF_SLOTFRAME_HANDLE_NEGOTIATED_CELLS);
  assert(slotframe != NULL);
}
/*---------------------------------------------------------------------------*/
void
msf_negotiated_cell_deactivate(void)
{
  slotframe = NULL;
}
/*---------------------------------------------------------------------------*/
tsch_slotframe_t *
msf_negotiated_cell_get_slotframe(void)
{
  return slotframe;
}
/*---------------------------------------------------------------------------*/
int
msf_negotiated_cell_add(msf_negotiated_cell_type_t type,
                        const linkaddr_t *peer_addr,
                        uint16_t slot_offset, uint16_t channel_offset)
{
  tsch_neighbor_t *nbr;
  uint8_t cell_options;
  const char* cell_type_str;
  tsch_link_t *new_cell;

  assert(slotframe != NULL);
  assert(peer_addr != NULL);

  if(type == MSF_NEGOTIATED_TX_CELL) {
    cell_options = LINK_OPTION_TX;
    cell_type_str = "TX";
  } else {
    assert(type == MSF_NEGOTIATED_RX_CELL);
    cell_options = LINK_OPTION_RX;
    cell_type_str = "RX";
  }

  if((nbr = tsch_queue_add_nbr(peer_addr)) == NULL &&
     (nbr = tsch_queue_add_nbr(peer_addr)) == NULL) {
    LOG_ERR("failed to add a negotiated %s cell because nbr is not available\n",
            cell_type_str);
    return -1;
  }

  new_cell = tsch_schedule_add_link(slotframe, cell_options,
                                    LINK_TYPE_NORMAL, peer_addr,
                                    slot_offset, channel_offset);

  if(new_cell != NULL && type == MSF_NEGOTIATED_TX_CELL) {
    if((new_cell->data = memb_alloc(&msf_negotiated_cell_data_memb)) == NULL) {
      LOG_ERR("memb_alloc failed for a new negotiated cell\n");
      (void)tsch_schedule_remove_link(slotframe, new_cell);
      new_cell = NULL;
    } else {
      memset(new_cell->data, 0, sizeof(msf_negotiated_cell_data_t));
    }
  }

  if(new_cell == NULL) {
    LOG_ERR("failed to add a negotiated %s cell for ", cell_type_str);
    LOG_ERR_LLADDR(peer_addr);
    LOG_ERR_(" at slot_offset:%u, channel_offset:%u\n",
             slot_offset, channel_offset);
  } else {
    LOG_INFO("added a negotiated %s cell for ", cell_type_str);
    LOG_INFO_LLADDR(peer_addr);
    LOG_INFO_(" at slot_offset:%u, channel_offset:%u\n",
              slot_offset, channel_offset);
    if(type == MSF_NEGOTIATED_RX_CELL) {
      /* MSF_NEGOTIATED_RX_CELL */
      /* we don't make a list for negotiated RX cells */
    } else {
      /* MSF_NEGOTIATED_TX_CELL */
      /* sort the list in reverse order of slot_offset */
      tsch_link_t *curr = nbr->negotiated_tx_cell;
      tsch_link_t *prev = NULL;
      while(curr != NULL) {
        if(new_cell->timeslot > curr->timeslot) {
          ((msf_negotiated_cell_data_t *)new_cell->data)->next = curr;
          if(prev == NULL) {
            /* new_cell has the largest slot_offset in the list */
            nbr->negotiated_tx_cell = new_cell;
          } else {
            ((msf_negotiated_cell_data_t *)prev->data)->next = new_cell;
          }
          break;
        } else {
          prev = curr;
          curr = ((msf_negotiated_cell_data_t *)curr->data)->next;
        }
      }
      if(curr == NULL) {
        if(nbr->negotiated_tx_cell == NULL) {
          nbr->negotiated_tx_cell = new_cell;
        } else {
          /* put the new cell to the end */
          ((msf_negotiated_cell_data_t *)prev->data)->next = new_cell;
        }
      }
    }
  }

  return new_cell == NULL ? -1 : 0;
}
/*---------------------------------------------------------------------------*/
void
msf_negotiated_cell_delete(tsch_link_t *cell)
{
  linkaddr_t peer_addr;
  const char *cell_type_str;
  tsch_link_t *curr_cell, *prev_cell;
  uint16_t slot_offset, channel_offset;

  assert(slotframe != NULL);
  assert(cell != NULL);
  linkaddr_copy(&peer_addr, &cell->addr);

  if(cell->link_options == LINK_OPTION_TX) {
    tsch_neighbor_t *nbr;
    cell_type_str = "TX";

    nbr = tsch_queue_get_nbr(&peer_addr);
    assert(nbr != NULL);
    assert(nbr->negotiated_tx_cell != NULL);
    /* update the chain of the negotiated cells */
    for(curr_cell = nbr->negotiated_tx_cell, prev_cell = NULL;
        curr_cell != NULL;
        curr_cell = ((msf_negotiated_cell_data_t *)curr_cell->data)->next) {
      if(curr_cell == cell) {
        if(prev_cell == NULL) {
          nbr->negotiated_tx_cell =
            ((msf_negotiated_cell_data_t *)cell->data)->next;
        } else {
          ((msf_negotiated_cell_data_t *)prev_cell->data)->next =
            ((msf_negotiated_cell_data_t *)cell->data)->next;
        }
        memb_free(&msf_negotiated_cell_data_memb, cell->data);
        break;
      } else {
        prev_cell = curr_cell;
      }
    }
  } else {
    assert(cell->link_options == LINK_OPTION_RX);
    cell_type_str = "RX";
  }

  slot_offset = cell->timeslot;
  channel_offset = cell->channel_offset;
  msf_housekeeping_delete_cell_later(cell);
  LOG_INFO("removed a negotiated %s cell for ", cell_type_str);
  LOG_INFO_LLADDR(&peer_addr);
  LOG_INFO_(" at slot_offset:%u, channel_offset:%u\n",
            slot_offset, channel_offset);
}
/*---------------------------------------------------------------------------*/
void
msf_negotiated_cell_delete_all(const linkaddr_t *peer_addr)
{
  tsch_link_t *cell;
  assert(slotframe);

  if(peer_addr == NULL) {
    while((cell = list_head(slotframe->links_list)) != NULL) {
      msf_negotiated_cell_delete(cell);
    }
  } else {
    tsch_neighbor_t *nbr;
    tsch_link_t *next_cell;
    if((nbr = tsch_queue_get_nbr(peer_addr)) != NULL) {
      for(cell = list_head(slotframe->links_list);
          cell != NULL;
          cell = next_cell) {
        next_cell = list_item_next(cell);
        if(linkaddr_cmp(&cell->addr, peer_addr)) {
          msf_negotiated_cell_delete(cell);
        } else {
          /* this cell is not scheduled with peer_addr; ignore it */
          continue;
        }
      }
      nbr->negotiated_tx_cell = NULL;
    }
  }
}
/*---------------------------------------------------------------------------*/
bool
msf_negotiated_cell_is_scheduled_tx(tsch_neighbor_t *nbr)
{
  assert(nbr != NULL);
  return nbr->negotiated_tx_cell != NULL;
}
/*---------------------------------------------------------------------------*/
tsch_link_t *
msf_negotiated_cell_get_tx_cell(tsch_neighbor_t *nbr)
{
  assert(nbr != NULL);
  return nbr->negotiated_tx_cell;
}
/*---------------------------------------------------------------------------*/
unsigned int
msf_negotiated_cell_get_num_tx_cells(const linkaddr_t *peer_addr)
{
  tsch_neighbor_t *nbr;
  int ret;

  if(peer_addr == NULL) {
    nbr = NULL;
  } else {
    nbr = tsch_queue_get_nbr(peer_addr);
  }

  if(nbr == NULL) {
    ret = 0;
  } else {
    ret = 0;
    for(tsch_link_t *cell = nbr->negotiated_tx_cell;
        cell != NULL;
        cell = ((msf_negotiated_cell_data_t *)cell->data)->next) {
      ret++;
    }
  }

  return ret;
}
/*---------------------------------------------------------------------------*/
void
msf_negotiated_cell_update_num_tx(uint16_t slot_offset,
                                  uint16_t num_tx, uint8_t mac_tx_status)
{
  /* update NumTx/NumTxAck of TX cells scheduled with the parent */
  const linkaddr_t *parent_addr;
  tsch_neighbor_t *nbr;

  if((parent_addr = msf_housekeeping_get_parent_addr()) == NULL ||
     (nbr = tsch_queue_get_nbr(parent_addr)) == NULL ||
     nbr->negotiated_tx_cell == NULL) {
    /* nothing to do */
  } else {
    tsch_link_t *cell;
    tsch_link_t *last_used = NULL;
    /* identify the cell that is used for the last transmission */
    for(cell = nbr->negotiated_tx_cell;
        cell != NULL;
        cell = ((msf_negotiated_cell_data_t *)cell->data)->next) {
      if(cell->timeslot == slot_offset) {
        last_used = cell;
        break;
      }
    }

    if(last_used == NULL) {
      /*
       * The last used cell is not scheduled as a negotiated cell for
       * the neighbor; give up updating NumTX/NumTxAck
       */
      return;
    } else {
      uint16_t i;
      msf_negotiated_cell_data_t *cell_data;

      /* update NumTxAck */
      if(mac_tx_status == MAC_TX_OK) {
        ((msf_negotiated_cell_data_t *)last_used->data)->num_tx_ack++;
      }

      /* update the counters */
      cell = last_used;
      for(i = 0; i < num_tx; i++) {
        assert(cell->data != NULL);
        cell_data = (msf_negotiated_cell_data_t *)cell->data;

        if(cell_data->num_tx == 255) {
          cell_data->num_tx = 128;
          cell_data->num_tx_ack /= 2;
        } else {
          cell_data->num_tx++;
        }

        if(((msf_negotiated_cell_data_t *)cell->data)->next == NULL) {
          cell = nbr->negotiated_tx_cell;
        } else {
          cell = ((msf_negotiated_cell_data_t *)cell->data)->next;
        }
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
tsch_link_t *
msf_negotiated_cell_get_cell_to_relocate(void)
{
  const linkaddr_t *parent_addr = msf_housekeeping_get_parent_addr();
  tsch_neighbor_t *nbr;
  tsch_link_t *cell;
  tsch_link_t *cell_to_relocate = NULL;
  msf_negotiated_cell_data_t *cell_data;
  tsch_link_t *worst_pdr_cell = NULL;
  int16_t best_pdr = -1; /* initialized with an invalid value for PDR */
  uint16_t worst_pdr = 100;
  int16_t pdr;

  assert(parent_addr != NULL);
  nbr = tsch_queue_get_nbr(parent_addr);
  assert(nbr != NULL);

  for(cell = nbr->negotiated_tx_cell; cell != NULL; cell = cell_data->next) {
    cell_data = (msf_negotiated_cell_data_t *)cell->data;
    assert(cell_data != NULL);
    if(cell_data->num_tx < MSF_MIN_NUM_TX_FOR_RELOCATION) {
      /* we don't evaluate this cell since it's not used much enough yet */
      pdr = -1;
    } else {
      assert(cell_data->num_tx > 0);
      pdr = cell_data->num_tx_ack * 100 / cell_data->num_tx;

      if(best_pdr < 0 || pdr > best_pdr) {
        best_pdr = pdr;
      }

      if(worst_pdr_cell == NULL || pdr < worst_pdr) {
        worst_pdr_cell = cell;
        worst_pdr = pdr;
        assert(best_pdr >= worst_pdr);
      }
    }
    LOG_DBG("cell[slot_offset: %3u, channel_offset: %3u] -- ",
            cell->timeslot, cell->channel_offset);
    LOG_DBG_("NumTx: %u, NumTxAck: %u ",
             cell_data->num_tx, cell_data->num_tx_ack);
    if(pdr < 0) {
      LOG_DBG_("PDR: N/A\n");
    } else {
      LOG_DBG_("PDR: %d\%%n", pdr);
    }
  }

  if(best_pdr < 0) {
    cell_to_relocate = NULL;
  } else {
    assert(worst_pdr >= 0);
    LOG_INFO("best PDR is %d%%, worst PDR is %u%%", best_pdr, worst_pdr);
    if((best_pdr - worst_pdr) <= MSF_RELOCATE_PDR_THRESHOLD) {
      /* worst_pdr_cell is not so bad to relocate */
      LOG_INFO_("\n");
      cell_to_relocate = NULL;
    } else {
      cell_to_relocate = worst_pdr_cell;
      LOG_INFO_("; going to relocate a TX cell"
                " [slot_offset: %u, channel_offset: %u]\n",
                cell_to_relocate->timeslot, cell_to_relocate->channel_offset);
    }
  }

  return cell_to_relocate;
}
/*---------------------------------------------------------------------------*/
uint16_t
msf_negotiated_cell_get_num_tx(tsch_link_t *cell)
{
  if(cell == NULL ||
     (cell->link_options & LINK_OPTION_TX) == 0 ||
     cell->data == NULL) {
    return 0;
  } else {
    return ((msf_negotiated_cell_data_t *)cell->data)->num_tx;
  }
}
/*---------------------------------------------------------------------------*/
uint16_t
msf_negotiated_cell_get_num_tx_ack(tsch_link_t *cell)
{
  if(cell == NULL ||
     (cell->link_options & LINK_OPTION_TX) == 0 ||
     cell->data == NULL) {
    return 0;
  } else {
    return ((msf_negotiated_cell_data_t *)cell->data)->num_tx_ack;
  }
}
/*---------------------------------------------------------------------------*/
