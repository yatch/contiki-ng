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
 *         MSF 6P-related common helpers (for internal use)
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#include "contiki.h"
#include "assert.h"

#include "lib/random.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/sixtop/sixp-pkt.h"

#include "msf.h"
#include "msf-negotiated-cell.h"
#include "msf-reserved-cell.h"
#include "msf-sixp.h"

#include "sys/log.h"
#define LOG_MODULE "MSF"
#define LOG_LEVEL LOG_LEVEL_MSF

/* variables */
static struct timer request_wait_timer;

/*---------------------------------------------------------------------------*/
const char *
msf_sixp_get_rc_str(sixp_pkt_rc_t rc)
{
  static const struct _rc_str_entry {
    sixp_pkt_rc_t rc;
    const char *str;
  } rc_str_list[] = {
    {SIXP_PKT_RC_SUCCESS, "RC_SUCCESS"},
    {SIXP_PKT_RC_EOL, "RC_EOL"},
    {SIXP_PKT_RC_ERR, "RC_ERR"},
    {SIXP_PKT_RC_RESET, "RC_RESET"},
    {SIXP_PKT_RC_ERR_VERSION, "RC_ERR_VERSION"},
    {SIXP_PKT_RC_ERR_SFID, "RC_ERR_SFID"},
    {SIXP_PKT_RC_ERR_SEQNUM, "RC_ERR_SEQNUM"},
    {SIXP_PKT_RC_ERR_CELLLIST, "RC_ERR_CELLLIST"},
    {SIXP_PKT_RC_ERR_BUSY, "RC_ERR_BUSY"},
    {SIXP_PKT_RC_ERR_LOCKED, "RC_ERR_LOCKED"},
  };
  static const char na[] = "N/A";
  const char *ret = na;
  for(int i = 0; i < sizeof(rc_str_list); i++) {
    if(rc_str_list[i].rc == rc) {
      ret = rc_str_list[i].str;
      break;
    }
  }
  return ret;
}
/*---------------------------------------------------------------------------*/
void
msf_sixp_set_cell_params(uint8_t *buf, const tsch_link_t *cell)
{
  if(buf == NULL || cell == NULL) {
    /* do nothing */
  } else {
    buf[0] = cell->timeslot & 0xff;
    buf[1] = cell->timeslot >> 8;
    buf[2] = cell->channel_offset & 0xff;
    buf[3] = cell->channel_offset >> 8;
  }
}
/*---------------------------------------------------------------------------*/
void
msf_sixp_get_cell_params(const uint8_t *buf,
                         uint16_t *slot_offset, uint16_t *channel_offset)
{
  if(buf == NULL || slot_offset == NULL || channel_offset == NULL) {
    /* do nothing */
  } else {
    *slot_offset = buf[0] + (buf[1] << 8);
    *channel_offset = buf[2] + (buf[3] << 8);
  }
}
/*---------------------------------------------------------------------------*/
bool
msf_sixp_is_request_wait_timer_expired(void)
{
  return timer_expired(&request_wait_timer);
}
/*---------------------------------------------------------------------------*/
void
msf_sixp_stop_request_wait_timer(void)
{
  timer_set(&request_wait_timer, 0);
}
/*---------------------------------------------------------------------------*/
void
msf_sixp_start_request_wait_timer(void)
{
  clock_time_t wait_duration_seconds;
  unsigned short random_value = random_rand();

  assert(MSF_WAIT_DURATION_MIN_SECONDS < MSF_WAIT_DURATION_MAX_SECONDS);
  wait_duration_seconds = (MSF_WAIT_DURATION_MIN_SECONDS +
                           ((MSF_WAIT_DURATION_MAX_SECONDS -
                             MSF_WAIT_DURATION_MIN_SECONDS) *
                            random_value /
                            RANDOM_RAND_MAX));

  assert(timer_expired(&request_wait_timer) != 0);
  timer_set(&request_wait_timer, wait_duration_seconds * CLOCK_SECOND);
  LOG_DBG("delay the next request for %lu seconds\n", wait_duration_seconds);
}
/*---------------------------------------------------------------------------*/
size_t
msf_sixp_fill_cell_list(const linkaddr_t *peer_addr,
                        msf_negotiated_cell_type_t cell_type,
                        uint8_t *cell_list, size_t cell_list_len)
{
  tsch_link_t *reserved_cell;
  size_t filled_cell_list_len = 0;
  while(filled_cell_list_len < cell_list_len) {
    reserved_cell = msf_reserved_cell_add(peer_addr, cell_type, -1, -1);
    if(reserved_cell == NULL) {
      break;
    } else {
      msf_sixp_set_cell_params(cell_list + filled_cell_list_len, reserved_cell);
      filled_cell_list_len += sizeof(sixp_pkt_cell_t);
    }
  }
  return filled_cell_list_len;
}
/*---------------------------------------------------------------------------*/
tsch_link_t *
msf_sixp_reserve_one_cell(const linkaddr_t *peer_addr,
                          msf_negotiated_cell_type_t cell_type,
                          const uint8_t *cell_list, size_t cell_list_len)
{
  size_t offset;
  uint16_t slot_offset, channel_offset;
  tsch_link_t *reserved_cell;

  if(peer_addr == NULL || cell_list == NULL) {
    reserved_cell = NULL;
  } else {
    for(offset = 0, reserved_cell = NULL;
        offset < cell_list_len;
        offset += sizeof(sixp_pkt_cell_t)) {
      msf_sixp_get_cell_params(cell_list + offset,
                               &slot_offset, &channel_offset);
      if((reserved_cell = msf_reserved_cell_add(peer_addr,
                                                cell_type,
                                                slot_offset,
                                                channel_offset)) != NULL) {
        break;
      } else {
        /*
         * cannot reserve a cell (most probably, this slot_offset is
         * occupied; try the next one
         */
      }
    }
  }
  return reserved_cell;
}
/*---------------------------------------------------------------------------*/
tsch_link_t *
msf_sixp_find_scheduled_cell(const linkaddr_t *peer_addr,
                             uint8_t link_options,
                             const uint8_t *cell_list, size_t cell_list_len)
{
  tsch_slotframe_t *slotframe = msf_negotiated_cell_get_slotframe();
  tsch_link_t *cell = NULL;

  if(peer_addr == NULL ||
     cell_list == NULL ||
     cell_list_len < sizeof(sixp_pkt_cell_t) ||
     slotframe == NULL) {
    /* invalid contion */
  } else {
    uint16_t slot_offset, channel_offset;

    for(const uint8_t *p = cell_list;
        p < cell_list + cell_list_len;
        p += sizeof(sixp_pkt_cell_t)) {

      msf_sixp_get_cell_params(cell_list, &slot_offset, &channel_offset);
      cell = tsch_schedule_get_link_by_timeslot(slotframe, slot_offset);

      if(cell != NULL ||
         cell->link_options == link_options ||
         cell->channel_offset == channel_offset ||
         linkaddr_cmp(&cell->addr, peer_addr)) {
        /* found */
        break;
      } else {
        /* try next */
      }
    }
  }

  return cell;
}
/*---------------------------------------------------------------------------*/
