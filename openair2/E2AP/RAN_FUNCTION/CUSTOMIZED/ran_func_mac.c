/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.1  (the "License"); you may not use this file
 * except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */

#include "ran_func_mac.h"
#include <assert.h>
#include <stdio.h>
#include <inttypes.h>
#include "common/ran_context.h"
#include "PHY/defs_gNB.h"

static
const int mod_id = 0;

#define MSR_RAPL_POWER_UNIT        0x606
#define MSR_PKG_ENERGY_STATUS      0x611
#define MSR_PP0_ENERGY_STATUS      0x639
#define MSR_DRAM_ENERGY_STATUS     0x619


bool read_mac_sm(void* data)
{
  assert(data != NULL);

  mac_ind_data_t* mac = (mac_ind_data_t*)data;
  //fill_mac_ind_data(mac);

  mac->msg.tstamp = time_now_us();
  PHY_VARS_gNB *gNB;
  gNB = RC.gNB[0];
  
  
  NR_UEs_t *UE_info = &RC.nrmac[mod_id]->UE_info;
  size_t num_ues = 0;
  UE_iterator(UE_info->list, ue) {
    if (ue)
      num_ues += 1;
  }

  mac->msg.len_ue_stats = num_ues;
  if(mac->msg.len_ue_stats > 0){
    mac->msg.ue_stats = calloc(mac->msg.len_ue_stats, sizeof(mac_ue_stats_impl_t));
    assert(mac->msg.ue_stats != NULL && "Memory exhausted" );
  }
  
  
  size_t i = 0; //TODO
  UE_iterator(UE_info->list, UE) {
    const NR_UE_sched_ctrl_t* sched_ctrl = &UE->UE_sched_ctrl;
    mac_ue_stats_impl_t* rd = &mac->msg.ue_stats[i];
    float poor_sched_rate = UE->mac_stats.ul.prate;
    
    NR_bler_options_t *ul_bler_options = &RC.nrmac[mod_id]->ul_bler;
    
    //NR_UE_UL_BWP_t *current_BWP = &UE->current_UL_BWP;
    //const uint16_t bwpSize = current_BWP->BWPSize;
    
    rd->rnti = UE->rnti;
    rd->frame = RC.nrmac[mod_id]->frame;
    rd->slot = RC.nrmac[mod_id]->slot;

    rd->pwr = (float)gNB->fpga_extra_energy;// + energy;    
    
    //gNB->fpga_extra_energy = 0.0;
       
    if (is_xlsch_in_slot(RC.nrmac[mod_id]->dlsch_slot_bitmap[rd->slot / 64], rd->slot)) {
      rd->dl_curr_tbs = UE->mac_stats.dl.current_bytes << 3;
      rd->dl_sched_rb = UE->mac_stats.dl.current_rbs;
    }
    if (is_xlsch_in_slot(RC.nrmac[mod_id]->ulsch_slot_bitmap[rd->slot / 64], rd->slot)) {
      rd->ul_curr_tbs = UE->mac_stats.ul.current_tbs << 3;
      rd->ul_sched_rb = UE->mac_stats.ul.current_rbs;
    }
    
    rd->pusch_snr = (float) sched_ctrl->pusch_snrx10 / 10; //: float = -64;
    rd->pucch_snr = (float) sched_ctrl->pucch_snrx10 / 10; //: float = -64;
     
    //const uint32_t bufferSize = sched_ctrl->estimated_ul_buffer - sched_ctrl->sched_ul_bytes;
    //rd->bsr = bufferSize;
    rd->bsr = UE->mac_stats.ul.current_demand;
    
    rd->wb_cqi = sched_ctrl->CSI_report.cri_ri_li_pmi_cqi_report.wb_cqi_1tb;
    rd->dl_mcs1 = sched_ctrl->dl_bler_stats.mcs;
    rd->dl_bler = sched_ctrl->dl_bler_stats.bler;
    rd->ul_mcs1 = sched_ctrl->ul_bler_stats.mcs;
    rd->ul_bler = sched_ctrl->ul_bler_stats.bler;
    rd->dl_mcs2 = 0;
    rd->ul_mcs2 = ul_bler_options->max_mcs;
        
    if (UE->mac_stats.ul.total_sched_blocks > 0){
    	poor_sched_rate = (float)UE->mac_stats.ul.poor_sched_blocks / UE->mac_stats.ul.total_sched_blocks;
    	//printf("sched_blocks %d, poor_blocks %d\n", UE->mac_stats.ul.total_sched_blocks, UE->mac_stats.ul.poor_sched_blocks);
    	UE->mac_stats.ul.poor_sched_blocks = 0;
    	UE->mac_stats.ul.total_sched_blocks = 0;
    }  
    float prate_filter = 0.9;
    UE->mac_stats.ul.prate = prate_filter * UE->mac_stats.ul.prate  + (1 - prate_filter) * poor_sched_rate;
    rd->poor_sched_rate = UE->mac_stats.ul.prate;
    ++i;
  }

  return num_ues > 0;
}

void read_mac_setup_sm(void* data)
{
  assert(data != NULL);
  assert(0 !=0 && "Not supported");
}

sm_ag_if_ans_t write_ctrl_mac_sm(void const* data)
{
  assert(data != NULL);
  //assert(0 !=0 && "Not supported");
  mac_ctrl_req_data_t const* mac_req_ctrl = (mac_ctrl_req_data_t const* )data;
  mac_ctrl_msg_t const* msg = &mac_req_ctrl->msg;
  NR_bler_options_t *ul_bler_options = &RC.nrmac[mod_id]->ul_bler;
  if (msg->ran_conf[0].isset_pusch_mcs){
  	ul_bler_options->max_mcs = msg->ran_conf[0].pusch_mcs;
  	RC.nrmac[mod_id]->max_prb = msg->ran_conf[0].pusch_prb;
  }
  
  //for (size_t i = 0; i < msg->ran_conf_len; i++) {
  // TODO
  //  printf("[E2-Agent]: ran_conf[%ld].isset_pusch_mcs %d\n", i, msg->ran_conf[i].isset_pusch_mcs);
  //  printf("[E2-Agent]: ran_conf[%ld].pusch_mcs %d\n", i, msg->ran_conf[i].pusch_mcs);
  //  printf("[E2-Agent]: ran_conf[%ld].rnti %d\n", i, msg->ran_conf[i].rnti);
  //}
  
  //printf("mcs: %u, prb: %u \n", ul_bler_options->max_mcs, RC.nrmac[mod_id]->max_prb);
  sm_ag_if_ans_t ans = {.type = CTRL_OUTCOME_SM_AG_IF_ANS_V0};
  ans.ctrl_out.type = MAC_AGENT_IF_CTRL_ANS_V0;
  ans.ctrl_out.mac.ans = MAC_CTRL_OUT_OK;
  return ans;
}

