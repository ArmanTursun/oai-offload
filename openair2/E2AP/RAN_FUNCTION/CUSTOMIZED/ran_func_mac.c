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

//static uint64_t read_msr(int fd, uint32_t reg) {
//    uint64_t data;
//    if (pread(fd, &data, sizeof(data), reg) != sizeof(data)) {
//        perror("pread");
//        exit(1);
//    }
//    return data;
//}

bool read_mac_sm(void* data)
{
  assert(data != NULL);

  mac_ind_data_t* mac = (mac_ind_data_t*)data;
  //fill_mac_ind_data(mac);

  mac->msg.tstamp = time_now_us();
  PHY_VARS_gNB *gNB;
  gNB = RC.gNB[0];
  
  //int fd;
  //char msr_file[32];
  //sprintf(msr_file, "/dev/cpu/0/msr");
  //fd = open(msr_file, O_RDONLY);
  //if (fd < 0) {
    //perror("open");
    //exit(1);
  //}  
  //gNB->pkg_energy_end = read_msr(fd, MSR_PKG_ENERGY_STATUS);
  //close(fd);
  
  //float energy = (gNB->pkg_energy_end - gNB->pkg_energy_start) * gNB->energy_unit;
  //gNB->pkg_energy_start = gNB->pkg_energy_end;
  
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
    //const NR_UE_sched_ctrl_t* sched_ctrl = &UE->UE_sched_ctrl;
    mac_ue_stats_impl_t* rd = &mac->msg.ue_stats[i];

    //NR_mac_ulsch_stats_t *ulsch_stats = &UE->mac_stats.ulsch_stats;
    //int rc_tbs = pthread_mutex_lock(&ulsch_stats->mutex);
    //DevAssert(rc_tbs == 0);

    //if(ulsch_stats->number_of_tbs > 0){
    	//rd->tbs = calloc(ulsch_stats->number_of_tbs, sizeof(mac_tbs_stats_t));
    	//assert(rd->tbs!= NULL && "Memory exhausted" );
    //}
    
    rd->rnti = UE->rnti;
    
    //rd->context.pusch_snr = (float) sched_ctrl->pusch_snrx10 / 10; //: float = -64;
    //rd->context.pucch_snr = (float) sched_ctrl->pusch_snrx10 / 10; //: float = -64;
    rd->dl_bler = (float)gNB->fpga_extra_energy;// + energy;    
    //rd->ul_bler = sched_ctrl->ul_bler_stats.bler;
    rd->ul_bler = (float)gNB->ldpc_latency;
    gNB->fpga_extra_energy = 0.0;
    gNB->ldpc_latency = 0.0;
    //printf("[E2 Agent Report]: fpga_energy = %f, energy = %f, bler: %f\n", (float)gNB->fpga_extra_energy, energy, sched_ctrl->ul_bler_stats.bler);
    //gNB->fpga_extra_energy = 0.0;
    //const uint32_t bufferSize = sched_ctrl->estimated_ul_buffer - sched_ctrl->sched_ul_bytes;
    //rd->context.bsr = bufferSize;
    //rd->context.wb_cqi = sched_ctrl->CSI_report.cri_ri_li_pmi_cqi_report.wb_cqi_1tb;
    //rd->context.dl_mcs1 = sched_ctrl->dl_bler_stats.mcs;
    //rd->context.ul_mcs1 = sched_ctrl->ul_bler_stats.mcs;
    //rd->context.dl_mcs2 = 0;
    //rd->context.ul_mcs2 = 0;
    //rd->context.phr = sched_ctrl->ph;
    
    // TODO add tbs to each UE
    //NR_mac_ulsch_stats_t *ulsch_stats = &UE->mac_stats.ulsch_stats;
    //rd->num_tbs = 0;
    //rd->tbs = calloc(1, sizeof(mac_tbs_stats_t));
    //assert(rd->tbs!= NULL && "Memory exhausted" );
    //rd->tbs[0].latency = gNB->fpga_extra_energy;
    //ulsch_stats->number_of_tbs = 0;   // dummy method to prevent from fulling
    //ulsch_stats->tbs_list[0].latency = 0;
    //int rc_tbs = pthread_mutex_lock(&ulsch_stats->mutex);
    //DevAssert(rc_tbs == 0);
/*
    if(ulsch_stats->number_of_tbs > 0){
    	rd->tbs = calloc(ulsch_stats->number_of_tbs, sizeof(mac_tbs_stats_t));
    	assert(rd->tbs!= NULL && "Memory exhausted" );
    }
    
    rd->num_tbs = ulsch_stats->number_of_tbs;
    for (int tbs_id = 0; tbs_id < ulsch_stats->number_of_tbs; tbs_id++){
      ulsch_tbs_stats_t *ulsch_tbs = &ulsch_stats->tbs_list[tbs_id];
      rd->tbs[tbs_id].tbs = ulsch_tbs->tbs;
      rd->tbs[tbs_id].frame = ulsch_tbs->frame;
      rd->tbs[tbs_id].slot = ulsch_tbs->slot;
      rd->tbs[tbs_id].latency = ulsch_tbs->latency;
      rd->tbs[tbs_id].crc = ulsch_tbs->crc_check;
    }

    ulsch_stats->number_of_tbs = 0;
*/
    //rc_tbs = pthread_mutex_unlock(&ulsch_stats->mutex);
    //DevAssert(rc_tbs == 0);
    
    //rd->tbs = calloc(NUM_UES, sizeof(mac_tbs_stats_t));
    //assert(rd->tbs != NULL && "memory exhausted");
    
    //for (int j = 0; j < rd->num_tbs; j++)
    //{
    //  rd->tbs[j].tbs = abs(rand()%mod);
    //  rd->tbs[j].frame = abs(rand()%mod);
    //  rd->tbs[j].slot = abs(rand()%mod);
    //  rd->tbs[j].latency = abs(rand()%mod);
    //  rd->tbs[j].crc = abs(rand()%mod);
    //}
    

    ++i;
    //if (rd->ul_curr_tbs > 0)
    //printf("[E2 Agent Report]: TBS = %" PRIu64 ", Timestamp = %" PRId64 "\n", rd->ul_curr_tbs, mac->msg.tstamp);
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
  PHY_VARS_gNB *gNB;
  gNB = RC.gNB[0];
  NR_bler_options_t *ul_bler_options = &RC.nrmac[mod_id]->ul_bler;
  ul_bler_options->max_mcs = msg->mcs;
  RC.nrmac[mod_id]->max_prb = msg->prb;
  
  //for (size_t i = 0; i < msg->ran_conf_len; i++) {
  	//gNB->ldpc_offload = msg->ran_conf[i].pusch_mcs;
  //}	
  // TODO copy offload indication to each UE
  //for (uint32_t i = 0; i < msg->num_ues; i++){
  	//mac_ue_ctrl_t* ue = &msg->ues[i];
  	//ue->offload = 1;
    //gNB->ldpc_offload = ue->offload;
    //printf("[E2 Agent Control]: ldpc_offload = %f, Timestamp = %" PRId64 "\n", ue->offload, msg->tms);
  //}
  //gNB->ldpc_offload = msg->offload;
  printf("mcs: %u, prb: %u \n", ul_bler_options->max_mcs, RC.nrmac[mod_id]->max_prb);
  sm_ag_if_ans_t ans = {.type = CTRL_OUTCOME_SM_AG_IF_ANS_V0};
  ans.ctrl_out.type = MAC_AGENT_IF_CTRL_ANS_V0;
  ans.ctrl_out.mac.ans = MAC_CTRL_OUT_OK;
  return ans;
}

