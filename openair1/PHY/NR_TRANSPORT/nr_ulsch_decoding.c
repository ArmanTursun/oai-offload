/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.0  (the "License"); you may not use this file
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

/*! \file PHY/NR_TRANSPORT/nr_ulsch_decoding.c
* \brief Top-level routines for decoding  LDPC (ULSCH) transport channels from 38.212, V15.4.0 2018-12
* \author Ahmed Hussein
* \date 2019
* \version 0.1
* \company Fraunhofer IIS
* \email: ahmed.hussein@iis.fraunhofer.de
* \note
* \warning
*/


// [from gNB coding]
#include "PHY/defs_gNB.h"
#include "PHY/CODING/coding_extern.h"
#include "PHY/CODING/coding_defs.h"
#include "PHY/CODING/lte_interleaver_inline.h"
#include "PHY/CODING/nrLDPC_extern.h"
#include "PHY/NR_TRANSPORT/nr_transport_common_proto.h"
#include "PHY/NR_TRANSPORT/nr_transport_proto.h"
#include "PHY/NR_TRANSPORT/nr_ulsch.h"
#include "PHY/NR_TRANSPORT/nr_dlsch.h"
#include "SCHED_NR/sched_nr.h"
#include "SCHED_NR/fapi_nr_l1.h"
#include "defs.h"
#include "common/utils/LOG/vcd_signal_dumper.h"
#include "common/utils/LOG/log.h"
#include <syscall.h>
#include "common/utils/nr/nr_common.h"
#include "/home/nakaolab/rfnoc_test.ldpc/include/ldpc_rfnoc_wrapper.h"
#include "openair2/E2AP/flexric/src/util/time_now_us.h"
#include <inttypes.h>
//#define DEBUG_ULSCH_DECODING
//#define gNB_DEBUG_TRACE

#define OAI_UL_LDPC_MAX_NUM_LLR 27000//26112 // NR_LDPC_NCOL_BG1*NR_LDPC_ZMAX = 68*384
//#define DEBUG_CRC
#ifdef DEBUG_CRC
#define PRINT_CRC_CHECK(a) a
#else
#define PRINT_CRC_CHECK(a)
#endif

//extern double cpuf;

void free_gNB_ulsch(NR_gNB_ULSCH_t *ulsch, uint16_t N_RB_UL)
{

  uint16_t a_segments = MAX_NUM_NR_ULSCH_SEGMENTS_PER_LAYER*NR_MAX_NB_LAYERS;  //number of segments to be allocated

  if (N_RB_UL != 273) {
    a_segments = a_segments*N_RB_UL;
    a_segments = a_segments/273 +1;
  }

  if (ulsch->harq_process) {
    if (ulsch->harq_process->b) {
      free_and_zero(ulsch->harq_process->b);
      ulsch->harq_process->b = NULL;
    }
    for (int r = 0; r < a_segments; r++) {
      free_and_zero(ulsch->harq_process->c[r]);
      free_and_zero(ulsch->harq_process->d[r]);
    }
    free_and_zero(ulsch->harq_process->c);
    free_and_zero(ulsch->harq_process->d);
    free_and_zero(ulsch->harq_process->d_to_be_cleared);
    free_and_zero(ulsch->harq_process);
    ulsch->harq_process = NULL;
  }
}

NR_gNB_ULSCH_t new_gNB_ulsch(uint8_t max_ldpc_iterations, uint16_t N_RB_UL)
{

  uint16_t a_segments = MAX_NUM_NR_ULSCH_SEGMENTS_PER_LAYER*NR_MAX_NB_LAYERS;  //number of segments to be allocated

  if (N_RB_UL != 273) {
    a_segments = a_segments*N_RB_UL;
    a_segments = a_segments/273 +1;
  }

  uint32_t ulsch_bytes = a_segments * 1056; // allocated bytes per segment
  NR_gNB_ULSCH_t ulsch = {0};

  ulsch.max_ldpc_iterations = max_ldpc_iterations;
  ulsch.harq_pid = -1;
  ulsch.active = false;

  NR_UL_gNB_HARQ_t *harq = malloc16_clear(sizeof(*harq));
  init_abort(&harq->abort_decode);
  ulsch.harq_process = harq;
  harq->b = malloc16_clear(ulsch_bytes * sizeof(*harq->b));
  harq->c = malloc16_clear(a_segments * sizeof(*harq->c));
  harq->d = malloc16_clear(a_segments * sizeof(*harq->d));
  for (int r = 0; r < a_segments; r++) {
    harq->c[r] = malloc16_clear(8448 * sizeof(*harq->c[r]));
    harq->d[r] = malloc16_clear(68 * 384 * sizeof(*harq->d[r]));
  }
  harq->d_to_be_cleared = calloc(a_segments, sizeof(bool));
  AssertFatal(harq->d_to_be_cleared != NULL, "out of memory\n");
  return(ulsch);
}

static void nr_processULSegment(void *arg)
{
  ldpcDecode_t *rdata = (ldpcDecode_t *)arg;
  NR_UL_gNB_HARQ_t *ulsch_harq = rdata->ulsch_harq;
  t_nrLDPC_dec_params *p_decoderParms = &rdata->decoderParms;
  const int Kr = ulsch_harq->K;
  const int Kr_bytes = Kr >> 3;
  const int K_bits_F = Kr - ulsch_harq->F;
  const int r = rdata->segment_r;
  const int A = rdata->A;
  const int E = rdata->E;
  const int Qm = rdata->Qm;
  const int rv_index = rdata->rv_index;
  const int r_offset = rdata->r_offset;
  const uint8_t kc = rdata->Kc;
  short *ulsch_llr = rdata->ulsch_llr;
  const int max_ldpc_iterations = p_decoderParms->numMaxIter;
  int8_t llrProcBuf[OAI_UL_LDPC_MAX_NUM_LLR] __attribute__((aligned(32)));

  t_nrLDPC_time_stats procTime = {0};
  t_nrLDPC_time_stats *p_procTime = &procTime;

  ////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////// nr_deinterleaving_ldpc ///////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////

  //////////////////////////// ulsch_llr =====> ulsch_harq->e //////////////////////////////

  /// code blocks after bit selection in rate matching for LDPC code (38.212 V15.4.0 section 5.4.2.1)
  int16_t harq_e[E];

  nr_deinterleaving_ldpc(E, Qm, harq_e, ulsch_llr + r_offset);

  // for (int i =0; i<16; i++)
  //          printf("rx output deinterleaving w[%d]= %d r_offset %d\n", i,ulsch_harq->w[r][i], r_offset);


  //////////////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////// nr_rate_matching_ldpc_rx ////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////// ulsch_harq->e =====> ulsch_harq->d /////////////////////////


  if (nr_rate_matching_ldpc_rx(rdata->tbslbrm,
                               p_decoderParms->BG,
                               p_decoderParms->Z,
                               ulsch_harq->d[r],
                               harq_e,
                               ulsch_harq->C,
                               rv_index,
                               ulsch_harq->d_to_be_cleared[r],
                               E,
                               ulsch_harq->F,
                               Kr - ulsch_harq->F - 2 * (p_decoderParms->Z))
      == -1) {

    LOG_E(PHY, "ulsch_decoding.c: Problem in rate_matching\n");
    rdata->decodeIterations = max_ldpc_iterations + 1;
    return;
  }

  ulsch_harq->d_to_be_cleared[r] = false;

  memset(ulsch_harq->c[r], 0, Kr_bytes);
  p_decoderParms->crc_type = crcType(ulsch_harq->C, A);
  p_decoderParms->E = lenWithCrc(ulsch_harq->C, A);
  // start_meas(&phy_vars_gNB->ulsch_ldpc_decoding_stats);

  // set first 2*Z_c bits to zeros

  int16_t z[68 * 384 + 16] __attribute__((aligned(16)));

  memset(z, 0, 2 * ulsch_harq->Z * sizeof(*z));
  // set Filler bits
  memset(z + K_bits_F, 127, ulsch_harq->F * sizeof(*z));
  // Move coded bits before filler bits
  memcpy(z + 2 * ulsch_harq->Z, ulsch_harq->d[r], (K_bits_F - 2 * ulsch_harq->Z) * sizeof(*z));
  // skip filler bits
  memcpy(z + Kr, ulsch_harq->d[r] + (Kr - 2 * ulsch_harq->Z), (kc * ulsch_harq->Z - Kr) * sizeof(*z));
  // Saturate coded bits before decoding into 8 bits values
  simde__m128i *pv = (simde__m128i *)&z;
  int8_t l[68 * 384 + 16] __attribute__((aligned(16)));
  simde__m128i *pl = (simde__m128i *)&l;
  for (int i = 0, j = 0; j < ((kc * ulsch_harq->Z) >> 4) + 1; i += 2, j++) {
    pl[j] = simde_mm_packs_epi16(pv[i], pv[i + 1]);
  }
  //////////////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////// nrLDPC_decoder /////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////// pl =====> llrProcBuf //////////////////////////////////
  rdata->decodeIterations =
      ldpc_interface.LDPCdecoder(p_decoderParms, 0, 0, 0, l, llrProcBuf, p_procTime, &ulsch_harq->abort_decode);

  if (rdata->decodeIterations <= p_decoderParms->numMaxIter)
    memcpy(ulsch_harq->c[r],llrProcBuf,  Kr>>3);
}


simde__m128i scale_and_pack_int16(simde__m128i val1, simde__m128i val2) {
    // Scale by 0.25 by right-shifting (equivalent to dividing by 4)
    // Add 2 before shifting to round the result instead of truncating
    //val1 = simde_mm_srai_epi16(simde_mm_add_epi16(val1, simde_mm_set1_epi16(2)), 2);
    //val2 = simde_mm_srai_epi16(simde_mm_add_epi16(val2, simde_mm_set1_epi16(2)), 2);
    
    val1 = simde_mm_srai_epi16(simde_mm_add_epi16(val1, simde_mm_set1_epi16(0)), 0);
    val2 = simde_mm_srai_epi16(simde_mm_add_epi16(val2, simde_mm_set1_epi16(0)), 0);

    // Pack the scaled 16-bit integers back into 8-bit, with saturation
    return simde_mm_packs_epi16(val1, val2);
}


/*
simde__m128i scale_and_pack_int16(simde__m128i val1, simde__m128i val2) {
    // Scale by 0.75 using integer arithmetic, acknowledging some precision loss.
    // Multiplying by 3 then dividing by 4 to achieve a scale of 0.75.
    // Adding 1 before division for rounding instead of truncation.
    simde__m128i three = simde_mm_set1_epi16(3);
    val1 = simde_mm_mullo_epi16(val1, three);
    val1 = simde_mm_add_epi16(val1, simde_mm_set1_epi16(1)); // For rounding
    val1 = simde_mm_srai_epi16(val1, 2); // Divide by 4

    val2 = simde_mm_mullo_epi16(val2, three);
    val2 = simde_mm_add_epi16(val2, simde_mm_set1_epi16(1)); // For rounding
    val2 = simde_mm_srai_epi16(val2, 2); // Divide by 4

    // Pack the scaled 16-bit integers back into 8-bit with saturation
    return simde_mm_packs_epi16(val1, val2);
}
*/

simde__m128i invert_sign(simde__m128i packed_values) {
    // Use zero-subtraction for negation
    simde__m128i zero = simde_mm_setzero_si128();
    return simde_mm_sub_epi8(zero, packed_values);
}

simde__m128i saturate_values(simde__m128i values) {
    // Define boundary values for saturation
    simde__m128i max_val = simde_mm_set1_epi8(7);
    simde__m128i min_val = simde_mm_set1_epi8(-8);

    // Compare values against boundaries
    simde__m128i mask_greater = simde_mm_cmpgt_epi8(values, max_val);
    simde__m128i mask_less = simde_mm_cmplt_epi8(values, min_val);

    // Saturate values based on comparison results
    values = simde_mm_blendv_epi8(values, max_val, mask_greater); // If values > 7, set to 7
    values = simde_mm_blendv_epi8(values, min_val, mask_less);    // If values < -8, set to -8

    return values;
}


static void nr_processULRFNoC_preDecode(void *arg)
{
  ldpcDecode_t *rdata = (ldpcDecode_t *)arg;
  //PHY_VARS_gNB *phy_vars_gNB = rdata->gNB;
  NR_UL_gNB_HARQ_t *ulsch_harq = rdata->ulsch_harq;
  t_nrLDPC_dec_params *p_decoderParms = &rdata->decoderParms;
  const int Kr = ulsch_harq->K;
  //const int Kr_bytes = Kr >> 3;
  const int K_bits_F = Kr - ulsch_harq->F;
  const int r = rdata->segment_r;
  const int A = rdata->A;
  const int E = rdata->E;
  const int Qm = rdata->Qm;
  const int rv_index = rdata->rv_index;
  const int r_offset = rdata->r_offset;
  const uint8_t kc = rdata->Kc;
  short *ulsch_llr = rdata->ulsch_llr;
  const int max_ldpc_iterations = p_decoderParms->numMaxIter;
  
  void* wrapper_gnb = (rdata->gNB)->wrapper_gnb;
  int instance_id_gnb = (rdata->gNB)->instance_id_gnb;
  
  uint8_t bg_rfnoc = 0;
  uint32_t Kb = 0;
  int info_length = lenWithCrc(ulsch_harq->C, A);
  if (info_length>3840){
      bg_rfnoc = 0;
      Kb = 22;
  } else if (info_length>640){
      bg_rfnoc = 1;
      Kb = 10;
  }else if (info_length>560){
      bg_rfnoc = 2;
      Kb = 9;
  }else if (info_length>192){
      bg_rfnoc = 3;
      Kb = 8;
  }else{
      bg_rfnoc = 4;
      Kb = 6;
  }
  
  uint8_t z_set = getZset_rfnoc(p_decoderParms->Z);
  uint8_t z_j = getZj_rfnoc(p_decoderParms->Z, z_set);
  //decParams.z_set = z_set;
  //decParams.z_j = z_j;
  //decParams.Kb = Kb;
  //uint32_t num_requested_samples_de = floor((harq_process->K + 127) / 128) * 128 / 32;
  //decParams.max_schedule = 0;
  //decParams.bg_rfnoc = bg_rfnoc;
  //decParams.sc_idx = phy_vars_gNB->sc_idx;
  
  
  uint8_t mb = getmb_rfnoc(p_decoderParms->R, Kb);
  uint64_t params_de[2];
  uint64_t* params_de_ptr = &params_de[0];
  //uint64_t params_de[2] = {0};
  //printf("segment id r: %d\n", r);
  createDecoderParam_rfnoc(params_de_ptr, 0, mb, r, max_ldpc_iterations, true, true, false, true, 12, bg_rfnoc, z_set, z_j);
  //printf("Decode Params %016lx, %016lx \n", params_de_ptr[1], params_de_ptr[0]);
  uint32_t samples_to_decode = ((mb + Kb) * p_decoderParms->Z * 8 + 31) / 32;

  ////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////// nr_deinterleaving_ldpc ///////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////

  //////////////////////////// ulsch_llr =====> ulsch_harq->e //////////////////////////////

  /// code blocks after bit selection in rate matching for LDPC code (38.212 V15.4.0 section 5.4.2.1)
  int16_t harq_e[E];

  nr_deinterleaving_ldpc(E, Qm, harq_e, ulsch_llr + r_offset);

  //////////////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////// nr_rate_matching_ldpc_rx ////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////// ulsch_harq->e =====> ulsch_harq->d /////////////////////////


  if (nr_rate_matching_ldpc_rx(rdata->tbslbrm,
                               p_decoderParms->BG,
                               p_decoderParms->Z,
                               ulsch_harq->d[r],
                               harq_e,
                               ulsch_harq->C,
                               rv_index,
                               ulsch_harq->d_to_be_cleared[r],
                               E,
                               ulsch_harq->F,
                               Kr - ulsch_harq->F - 2 * (p_decoderParms->Z))
      == -1) {

    LOG_E(PHY, "ulsch_decoding.c: Problem in rate_matching\n");
    rdata->decodeIterations = max_ldpc_iterations + 1;
    return;
  }

  ulsch_harq->d_to_be_cleared[r] = false;

  // set first 2*Z_c bits to zeros

  int16_t z[68 * 384 + 16] __attribute__((aligned(16)));

  memset(z, 512, 2 * ulsch_harq->Z * sizeof(*z));
  // set Filler bits
  memset(z + K_bits_F, 512, ulsch_harq->F * sizeof(*z));
  // Move coded bits before filler bits
  memcpy(z + 2 * ulsch_harq->Z, ulsch_harq->d[r], (K_bits_F - 2 * ulsch_harq->Z) * sizeof(*z));
  // skip filler bits
  memcpy(z + Kr, ulsch_harq->d[r] + (Kr - 2 * ulsch_harq->Z), (kc * ulsch_harq->Z - Kr) * sizeof(*z));
  // Saturate coded bits before decoding into 8 bits values
  simde__m128i *pv = (simde__m128i *)&z;
  int8_t l[68 * 384 + 16] __attribute__((aligned(16)));
  simde__m128i *pl = (simde__m128i *)&l;
  
  //for (int i = 2 * ulsch_harq->Z; i < 2 * ulsch_harq->Z + 4; i++) printf("before scale=%d \n",z[i]);
/*  
  //simde__m128i max_val = simde_mm_set1_epi16(7);   // Set max value (7)
  //simde__m128i min_val = simde_mm_set1_epi16(-8);
  //simde__m128i zero = simde_mm_setzero_si128();    // Zero vector for sign inversion
  for (int i = 0, j = 0; j < ((kc * ulsch_harq->Z) >> 4) + 1; i += 2, j++) {
    // Scale by 0.25 and pack the results into 8-bit integers
    simde__m128i scaled_and_packed = scale_and_pack_int16(pv[i], pv[i + 1]);
    simde__m128i saturated_values = saturate_values(scaled_and_packed);
    // Invert the sign of the packed values
    simde__m128i inverted_sign_values = invert_sign(saturated_values);    
    // Store the saturated result in pl
    pl[j] = inverted_sign_values;
  }
*/  
  //for (int i = 2 * ulsch_harq->Z; i < 2 * ulsch_harq->Z + 4; i++) printf("after scale=%d \n",l[i]);
  
  
  simde__m128i max_val = simde_mm_set1_epi16(7);   // Set max value (7)
  simde__m128i min_val = simde_mm_set1_epi16(-8);  // Set min value (-8)
  simde__m128i zero = simde_mm_setzero_si128();    // Zero vector for sign inversion
  simde__m128i initial_value = simde_mm_set1_epi8(0x87);
  
  for (int i = 0, j = 0; j < ((kc * ulsch_harq->Z) >> 4) + 1; i += 2, j++) { 
    
    //if (j < 2 * ulsch_harq->Z / 16) {
    if (false) {
        pl[j] = initial_value;
    } else {
        // The rest of your original loop logic
        simde__m128i val1 = simde_mm_min_epi16(simde_mm_max_epi16(pv[i], min_val), max_val);
        simde__m128i val2 = simde_mm_min_epi16(simde_mm_max_epi16(pv[i + 1], min_val), max_val);
        
        // Invert sign by subtracting from zero
        simde__m128i val1_inv = simde_mm_sub_epi16(zero, val1);
        simde__m128i val2_inv = simde_mm_sub_epi16(zero, val2);
        
        pl[j] = simde_mm_packs_epi16(val1_inv, val2_inv);
        //pl[j] = simde_mm_packs_epi16(val1, val2);

    }    
  }

  //memset(l + K_bits_F, 0xE1, ulsch_harq->F * sizeof(*l));  
  Decoder_send(wrapper_gnb, instance_id_gnb, (uint32_t*)&l, &samples_to_decode, params_de_ptr);
}

/*
void getDecoderStatus2(uint64_t status, int* max_iterations, int* term_no_change, int* term_on_pass, int* parity_pass) {
    // Perform some computations to get the value

    // get de_iterations
    *max_iterations = (int)((status >> 18)  & 0x3f);

    // get term_on_no_change
    *term_no_change = (int)((status >> 17) & 1);

    // get term_on_pass
    *term_on_pass = (int)((status >> 16) & 1);

    // get parity_pass
    *parity_pass = (int)((status >> 15) & 1);
}
*/

void getDecoderStatus2(uint64_t status, int* max_iterations, int* term_no_change, int* term_on_pass, int* parity_pass, int* r) {
    // Perform some computations to get the value
    uint64_t status_;

    // get parity_pass
    status_ = status >> 15;
    *parity_pass = (int)(status_ & 1);

    // get term_on_pass
    status_ = status_ >> 1;
    *term_on_pass = (int)(status_ & 1);

    // get term_on_no_change
    status_ = status_ >> 1;
    *term_no_change = (int)(status_ & 1);

    // get de_iterations
    status_ = status_ >> 1;
    *max_iterations = (int)(status_  & 0x3f);

    // get de_iterations
    *r = (int)((status_ >> 6)  & 0xff);
}

static void nr_processULRFNoC_postDecode(void *arg)
{
  ldpcDecode_t *rdata = (ldpcDecode_t *)arg;
  NR_UL_gNB_HARQ_t *ulsch_harq = rdata->ulsch_harq;
  t_nrLDPC_dec_params *p_decoderParms = &rdata->decoderParms;
  const int Kr = ulsch_harq->K;
  const int Kr_bytes = Kr >> 3;

  const int A = rdata->A;
  uint8_t llrProcBuf[OAI_UL_LDPC_MAX_NUM_LLR] __attribute__((aligned(32)));
  
  void* wrapper_gnb = (rdata->gNB)->wrapper_gnb;
  int instance_id_gnb = (rdata->gNB)->instance_id_gnb;
  
  uint64_t status_de[2];
  uint64_t* status_de_ptr = &status_de[0];
  
  uint32_t Kb = 0;
  int info_length = lenWithCrc(ulsch_harq->C, A);
  if (info_length>3840){
      Kb = 22;
  } else if (info_length>640){
      Kb = 10;
  }else if (info_length>560){
      Kb = 9;
  }else if (info_length>192){
      Kb = 8;
  }else{
      Kb = 6;
  }
  
  uint32_t num_requested_samples_de = floor((p_decoderParms->Z * Kb + 127) / 128) * 128 / 32;
  
  Decoder_recv(wrapper_gnb, instance_id_gnb, &num_requested_samples_de, (uint32_t *)llrProcBuf, status_de_ptr);
  
  //if (BG==1){
  if (true){
      for (int aa = 0; aa < Kr>>3; aa++){
          llrProcBuf[aa] = reverseBits_rfnoc(llrProcBuf[aa]);
      }
  }
  
  //printf("Decode Status %016lx, %016lx \n", status_de_ptr[1], status_de_ptr[0]);
  int de_iterations, term_no_change, term_on_pass, parity_pass;
  getDecoderStatus2(status_de_ptr[1], &de_iterations, &term_no_change, &term_on_pass, &parity_pass, &rdata->current_seg);
  //printf("De_iterations %d, term_no_change %d, term_on_pass %d, parity_pass %d, r %d \n", de_iterations, term_no_change, term_on_pass, parity_pass, rdata->current_seg);
  rdata->decodeIterations = de_iterations;
  int r_offset = rdata->E * rdata->current_seg;
  int offset = (Kr_bytes - (ulsch_harq->F >> 3) - ((ulsch_harq->C > 1) ? 3 : 0)) * rdata->current_seg;
  //printf("r_offset %d, offset %d, rdata_r_offset %d, rdata_offset %d\n", r_offset, offset, rdata->r_offset, rdata->offset);
  rdata->r_offset = r_offset;
  rdata->offset = offset;
  rdata->segment_r = rdata->current_seg;
  const int r = rdata->segment_r;
  //printf("r_offset %d, offset %d, rdata_r_offset %d, rdata_offset %d, r %d\n", r_offset, offset, rdata->r_offset, rdata->offset, r);
  //int r_ = getDecoderStatus_rfnoc(status_de_ptr, &(rdata->decodeIterations));
  //printf("segment id r_: %d, r: %d\n", r_, r);
  memset(ulsch_harq->c[r], 0, Kr_bytes);

  //for (int i = 0; i < 4; i++) printf("received_output[i]=%hhu \n",llrProcBuf[i]);
  //if (check_crc(llrProcBuf, lenWithCrc(ulsch_harq->C, A), crcType(ulsch_harq->C, A))) {
        //printf("Segment %d CRC OK\n", rdata->current_seg);
        //rdata->decodeIterations = 2;
  //} else {
        //printf("segment %d CRC NOK\n", rdata->current_seg);
        //rdata->decodeIterations = p_decoderParms->numMaxIter + 1;
  //}
  
  if (rdata->decodeIterations <= p_decoderParms->numMaxIter)
    memcpy(ulsch_harq->c[r],llrProcBuf,  Kr>>3);
  //printf("\n");
}



int decode_offload(PHY_VARS_gNB *phy_vars_gNB,
                   uint8_t ULSCH_id,
                   short *ulsch_llr,
                   nfapi_nr_pusch_pdu_t *pusch_pdu,
                   t_nrLDPC_dec_params *decParams,
                   uint8_t harq_pid,
                   uint32_t G)
{
  NR_gNB_ULSCH_t *ulsch = &phy_vars_gNB->ulsch[ULSCH_id];
  NR_UL_gNB_HARQ_t *harq_process = ulsch->harq_process;
  int16_t z_ol[LDPC_MAX_CB_SIZE] __attribute__((aligned(16)));
  int8_t l_ol[LDPC_MAX_CB_SIZE] __attribute__((aligned(16)));
  uint8_t Qm = pusch_pdu->qam_mod_order;
  uint8_t n_layers = pusch_pdu->nrOfLayers;
  const int Kr = harq_process->K;
  const int Kr_bytes = Kr >> 3;
  uint32_t A = (harq_process->TBS) << 3;
  const int kc = decParams->BG == 2 ? 52 : 68;
  ulsch->max_ldpc_iterations = 20;
  int decodeIterations = 2;
  int r_offset = 0, offset = 0;
  for (int r = 0; r < harq_process->C; r++) {
    int E = nr_get_E(G, harq_process->C, Qm, n_layers, r);
    memset(harq_process->c[r], 0, Kr_bytes);
    decParams->R = nr_get_R_ldpc_decoder(pusch_pdu->pusch_data.rv_index,
                                         E,
                                         decParams->BG,
                                         decParams->Z,
                                         &harq_process->llrLen,
                                         harq_process->round);

    memcpy(z_ol, ulsch_llr + r_offset, E * sizeof(short));
    simde__m128i *pv_ol128 = (simde__m128i *)&z_ol;
    simde__m128i *pl_ol128 = (simde__m128i *)&l_ol;
    for (int i = 0, j = 0; j < ((kc * harq_process->Z) >> 4) + 1; i += 2, j++) {
      pl_ol128[j] = simde_mm_packs_epi16(pv_ol128[i], pv_ol128[i + 1]);
    }
    decParams->E = E;
    decParams->rv = pusch_pdu->pusch_data.rv_index;
    decParams->F = harq_process->F;
    decParams->Qm = Qm;
    decodeIterations =
        ldpc_interface_offload
            .LDPCdecoder(decParams, harq_pid, ULSCH_id, r, (int8_t *)&pl_ol128[0], (int8_t *)harq_process->c[r], NULL, NULL);
    if (decodeIterations < 0) {
      LOG_E(PHY, "ulsch_decoding.c: Problem in LDPC decoder offload\n");
      return -1;
    }
    bool decodeSuccess = check_crc((uint8_t *)harq_process->c[r], lenWithCrc(harq_process->C, A), crcType(harq_process->C, A));
    if (decodeSuccess) {
      memcpy(harq_process->b + offset, harq_process->c[r], Kr_bytes - (harq_process->F >> 3) - ((harq_process->C > 1) ? 3 : 0));
      offset += (Kr_bytes - (harq_process->F >> 3) - ((harq_process->C > 1) ? 3 : 0));
      harq_process->processedSegments++;
    } else {
      LOG_D(PHY, "uplink segment error %d/%d\n", r, harq_process->C);
      LOG_D(PHY, "ULSCH %d in error\n", ULSCH_id);
    }
    r_offset += E;
  }
  bool crc_valid = false;
  if (harq_process->processedSegments == harq_process->C) {
    // When the number of code blocks is 1 (C = 1) and ulsch_harq->processedSegments = 1, we can assume a good TB because of the
    // CRC check made by the LDPC for early termination, so, no need to perform CRC check twice for a single code block
    crc_valid = true;
    if (harq_process->C > 1) {
      crc_valid = check_crc(harq_process->b, lenWithCrc(1, A), crcType(1, A));
    }
  }
  if (crc_valid) {
    LOG_D(PHY, "ULSCH: Setting ACK for slot %d TBS %d\n", ulsch->slot, harq_process->TBS);
    nr_fill_indication(phy_vars_gNB, ulsch->frame, ulsch->slot, ULSCH_id, harq_pid, 0, 0);
    ulsch->active = false;
    harq_process->round = 0;
  } else {
    LOG_D(PHY,
        "[gNB %d] ULSCH: Setting NAK for SFN/SF %d/%d (pid %d, status %d, round %d, TBS %d)\n",
        phy_vars_gNB->Mod_id,
        ulsch->frame,
        ulsch->slot,
        harq_pid,
        ulsch->active,
        harq_process->round,
        harq_process->TBS);
    nr_fill_indication(phy_vars_gNB, ulsch->frame, ulsch->slot, ULSCH_id, harq_pid, 1, 0);
    ulsch->handled = 1;
    decodeIterations = ulsch->max_ldpc_iterations + 1;
    LOG_D(PHY, "ULSCH %d in error\n", ULSCH_id);
  }

  ulsch->last_iteration_cnt = decodeIterations;
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_gNB_ULSCH_DECODING,0);
  return 0;
}

int nr_ulsch_decoding(PHY_VARS_gNB *phy_vars_gNB,
                      uint8_t ULSCH_id,
                      short *ulsch_llr,
                      NR_DL_FRAME_PARMS *frame_parms,
                      nfapi_nr_pusch_pdu_t *pusch_pdu,
                      uint32_t frame,
                      uint8_t nr_tti_rx,
                      uint8_t harq_pid,
                      uint32_t G)
{
  if (!ulsch_llr) {
    LOG_E(PHY, "ulsch_decoding.c: NULL ulsch_llr pointer\n");
    return -1;
  }

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_gNB_ULSCH_DECODING, 1);

  NR_gNB_ULSCH_t *ulsch = &phy_vars_gNB->ulsch[ULSCH_id];
  NR_gNB_PUSCH *pusch = &phy_vars_gNB->pusch_vars[ULSCH_id];
  NR_UL_gNB_HARQ_t *harq_process = ulsch->harq_process;

  if (!harq_process) {
    LOG_E(PHY, "ulsch_decoding.c: NULL harq_process pointer\n");
    return -1;
  }

  // ------------------------------------------------------------------
  const uint16_t nb_rb = pusch_pdu->rb_size;
  const uint8_t Qm = pusch_pdu->qam_mod_order;
  const uint8_t mcs = pusch_pdu->mcs_index;
  const uint8_t n_layers = pusch_pdu->nrOfLayers;
  // ------------------------------------------------------------------

  harq_process->processedSegments = 0;
  harq_process->TBS = pusch_pdu->pusch_data.tb_size;

  t_nrLDPC_dec_params decParams = {.check_crc = check_crc};
  decParams.BG = pusch_pdu->maintenance_parms_v3.ldpcBaseGraph;
  const uint32_t A = (harq_process->TBS) << 3;
  NR_gNB_PHY_STATS_t *stats = get_phy_stats(phy_vars_gNB, ulsch->rnti);
  if (stats) {
    stats->frame = frame;
    stats->ulsch_stats.round_trials[harq_process->round]++;
    for (int aarx = 0; aarx < frame_parms->nb_antennas_rx; aarx++) {
      stats->ulsch_stats.power[aarx] = dB_fixed_x10(pusch->ulsch_power[aarx]);
      stats->ulsch_stats.noise_power[aarx] = dB_fixed_x10(pusch->ulsch_noise_power[aarx]);
    }
    if (!harq_process->harq_to_be_cleared) {
      stats->ulsch_stats.current_Qm = Qm;
      stats->ulsch_stats.current_RI = n_layers;
      stats->ulsch_stats.total_bytes_tx += harq_process->TBS;
    }
  }

  LOG_D(PHY,
        "ULSCH Decoding, harq_pid %d rnti %x TBS %d G %d mcs %d Nl %d nb_rb %d, Qm %d, Coderate %f RV %d round %d new RX %d\n",
        harq_pid,
        ulsch->rnti,
        A,
        G,
        mcs,
        n_layers,
        nb_rb,
        Qm,
        pusch_pdu->target_code_rate / 10240.0f,
        pusch_pdu->pusch_data.rv_index,
        harq_process->round,
        harq_process->harq_to_be_cleared);

  // [hna] Perform nr_segmenation with input and output set to NULL to calculate only (C, K, Z, F)
  nr_segmentation(NULL,
                  NULL,
                  lenWithCrc(1, A), // size in case of 1 segment
                  &harq_process->C,
                  &harq_process->K,
                  &harq_process->Z, // [hna] Z is Zc
                  &harq_process->F,
                  decParams.BG);

  uint16_t a_segments = MAX_NUM_NR_ULSCH_SEGMENTS_PER_LAYER * n_layers; // number of segments to be allocated
  if (harq_process->C > a_segments) {
    LOG_E(PHY, "nr_segmentation.c: too many segments %d, A %d\n", harq_process->C, A);
    return(-1);
  }
  if (nb_rb != 273) {
    a_segments = a_segments*nb_rb;
    a_segments = a_segments/273 +1;
  }
  if (harq_process->C > a_segments) {
    LOG_E(PHY,"Illegal harq_process->C %d > %d\n",harq_process->C,a_segments);
    return -1;
  }

#ifdef DEBUG_ULSCH_DECODING
  printf("ulsch decoding nr segmentation Z %d\n", harq_process->Z);
  if (!frame % 100)
    printf("K %d C %d Z %d \n", harq_process->K, harq_process->C, harq_process->Z);
  printf("Segmentation: C %d, K %d\n",harq_process->C,harq_process->K);
#endif

  decParams.Z = harq_process->Z;
  decParams.numMaxIter = ulsch->max_ldpc_iterations;
  decParams.outMode = 0;
  decParams.setCombIn = !harq_process->harq_to_be_cleared;
  if (harq_process->harq_to_be_cleared) {
    for (int r = 0; r < harq_process->C; r++)
      harq_process->d_to_be_cleared[r] = true;
    harq_process->harq_to_be_cleared = false;
  }
  
  
  //int offload_flag = -1;
  //if (phy_vars_gNB->ldpc_offload >= 0)
  //  offload_flag = phy_vars_gNB->ldpc_offload;
  //if (offload_flag >= 0 )
  //printf("[E2-AGENT Offload]: ldpc_offload = %d, TBS = %d, Timestamp = %" PRId64 "\n", phy_vars_gNB->ldpc_offload, A, time_now_us());
  //printf("[E2-AGENT Offload]: ldpc_offload = %d, TBS = %d, ULSCH_id = %d, rnti = %d \n", phy_vars_gNB->ldpc_offload, A, ULSCH_id, ulsch->rnti);

  if (phy_vars_gNB->ldpc_offload_flag)
    return decode_offload(phy_vars_gNB, ULSCH_id, ulsch_llr, pusch_pdu, &decParams, harq_pid, G);

  uint32_t offset = 0, r_offset = 0;
  set_abort(&harq_process->abort_decode, false);
////////////////// RFNOC ////////////////////////
  //int no_iteration_ldpc[MAX_NUM_NR_DLSCH_SEGMENTS_PER_LAYER*NR_MAX_NB_LAYERS];


  if (decParams.BG == 1){
  //if (false){
  //if (A > 928){
  	//printf("*** Using FPGA decoding, TBS: %d, rv_index: %d, max_iteration: %d\n", A, pusch_pdu->pusch_data.rv_index, decParams.numMaxIter);
  
  	notifiedFIFO_t nf_rfnoc;
  	initNotifiedFIFO(&nf_rfnoc);
  	int jobs_pre = 0;
  	for (int r = 0; r < harq_process->C; r++) {
    		int E = nr_get_E(G, harq_process->C, Qm, n_layers, r);
    		union ldpcReqUnion id = {.s = {ulsch->rnti, frame, nr_tti_rx, 0, A}};
    		//union ldpcReqUnion id = {.s = {0, 0, 0, 0, A}};
            notifiedFIFO_elt_t *req = NULL;
            //if (r < harq_process->C / 2) {
            //    req = newNotifiedFIFO_elt(sizeof(ldpcDecode_t), id.s.spare,
            //                                                  &phy_vars_gNB->respDecode_pre_1,
            //                                                  &nr_processULRFNoC_preDecode);
            //    phy_vars_gNB->nbDecode_1++;
            //} else {
            //    req = newNotifiedFIFO_elt(sizeof(ldpcDecode_t), id.s.spare,
            //                                                  &phy_vars_gNB->respDecode_pre_2,
            //                                                  &nr_processULRFNoC_preDecode);
            //    phy_vars_gNB->nbDecode_2++;
            //}
            req = newNotifiedFIFO_elt(sizeof(ldpcDecode_t), id.s.spare, &phy_vars_gNB->respDecode_pre, &nr_processULRFNoC_preDecode);
    		ldpcDecode_t *rdata = (ldpcDecode_t *)NotifiedFifoData(req);
    		decParams.R = nr_get_R_ldpc_decoder(pusch_pdu->pusch_data.rv_index,
                                        E,
                                        decParams.BG,
                                        decParams.Z,
                                        &harq_process->llrLen,
                                        harq_process->round);
    
    		rdata->gNB = phy_vars_gNB;
    		rdata->ulsch_harq = harq_process;
    		rdata->decoderParms = decParams;
    		rdata->ulsch_llr = ulsch_llr;
    		rdata->Kc = decParams.BG == 2 ? 52 : 68;
    		rdata->harq_pid = harq_pid;
    		rdata->segment_r = r;
    		rdata->nbSegments = harq_process->C;
    		rdata->E = E;
    		rdata->A = A;
    		rdata->Qm = Qm;
    		rdata->r_offset = r_offset;
    		rdata->Kr_bytes = harq_process->K >> 3;
    		rdata->rv_index = pusch_pdu->pusch_data.rv_index;
    		rdata->offset = offset;
    		rdata->ulsch = ulsch;
    		rdata->ulsch_id = ULSCH_id;
    		//rdata->num_requested_samples_de = &num_requested_samples_de;
    		rdata->tbslbrm = pusch_pdu->maintenance_parms_v3.tbSizeLbrmBytes;
    		pushTpool(&phy_vars_gNB->threadPool, req);
            jobs_pre++;
    		LOG_D(PHY, "Added a block to decode, in pipe: %d\n", r);
    		r_offset += E;
    		offset += ((harq_process->K >> 3) - (harq_process->F >> 3) - ((harq_process->C > 1) ? 3 : 0));
  	}
/*
    //int decode_num = 0;
    while (phy_vars_gNB->nbDecode_1 > 0) {
          notifiedFIFO_elt_t *req = pullTpool(&phy_vars_gNB->respDecode_pre_1, &phy_vars_gNB->threadPool_LDPC_de);
          //notifiedFIFO_elt_t *req = pullTpool(&nf_rfnoc, &phy_vars_gNB->threadPool_LDPC_de);
          if (req == NULL)
              break; // Tpool has been stopped
          phy_vars_gNB->nbDecode_1--;
          delNotifiedFIFO_elt(req);
          //decode_num++;
    }
*/
  	while (jobs_pre > 0) {
          notifiedFIFO_elt_t *req = pullTpool(&phy_vars_gNB->respDecode_pre, &phy_vars_gNB->threadPool);
          //notifiedFIFO_elt_t *req = pullTpool(&nf_rfnoc, &phy_vars_gNB->threadPool_LDPC_de);
          if (req == NULL)
              break; // Tpool has been stopped
          jobs_pre--;
          delNotifiedFIFO_elt(req);
  	}

  	r_offset = 0;
  	offset = 0;
    //uint32_t num_requested_samples_de = floor((decParams.Z * Kb + 127) / 128) * 128 / 32;
  	for (int r = 0; r < harq_process->C; r++) {
  			//printf("requested samples in round %d: %d\n", r, num_requested_samples_de);
    		int E = nr_get_E(G, harq_process->C, Qm, n_layers, r);
    		union ldpcReqUnion id = {.s = {ulsch->rnti, frame, nr_tti_rx, 1, A}};
    		//union ldpcReqUnion id = {.s = {0, 0, 0, 0, A+1}};
            //notifiedFIFO_elt_t *req = NULL;
            //if (r < harq_process->C / 2){
            //    req = newNotifiedFIFO_elt(sizeof(ldpcDecode_t), id.s.spare, &phy_vars_gNB->respDecode_post_1, &nr_processULRFNoC_postDecode);
            //    phy_vars_gNB->nbDecode_1++;
            //} else {
            //    req = newNotifiedFIFO_elt(sizeof(ldpcDecode_t), id.s.spare, &phy_vars_gNB->respDecode_post_2, &nr_processULRFNoC_postDecode);
            //    phy_vars_gNB->nbDecode_2++;
            //}
    		notifiedFIFO_elt_t *req = newNotifiedFIFO_elt(sizeof(ldpcDecode_t), id.s.spare, &phy_vars_gNB->respDecode_post, &nr_processULRFNoC_postDecode);
    		ldpcDecode_t *rdata = (ldpcDecode_t *)NotifiedFifoData(req);
    		decParams.R = nr_get_R_ldpc_decoder(pusch_pdu->pusch_data.rv_index,
                                        E,
                                        decParams.BG,
                                        decParams.Z,
                                        &harq_process->llrLen,
                                        harq_process->round);
    		rdata->gNB = phy_vars_gNB;
    		rdata->ulsch_harq = harq_process;
    		rdata->decoderParms = decParams;
    		rdata->ulsch_llr = ulsch_llr;
    		rdata->Kc = decParams.BG == 2 ? 52 : 68;
    		rdata->harq_pid = harq_pid;
    		rdata->segment_r = r;
    		rdata->nbSegments = harq_process->C;
    		rdata->E = E;
    		rdata->A = A;
    		rdata->Qm = Qm;
    		rdata->r_offset = r_offset;
    		rdata->Kr_bytes = harq_process->K >> 3;
    		rdata->rv_index = pusch_pdu->pusch_data.rv_index;
    		rdata->offset = offset;
    		rdata->ulsch = ulsch;
    		rdata->ulsch_id = ULSCH_id;
            phy_vars_gNB->nbDecode_offload++;
    		rdata->tbslbrm = pusch_pdu->maintenance_parms_v3.tbSizeLbrmBytes;
    		//rdata->num_requested_samples_de = &num_requested_samples_de;
    		pushTpool(&phy_vars_gNB->threadPool, req);
    		LOG_D(PHY, "Added a block to decode, in pipe: %d\n", r);
    		r_offset += E;
    		offset += ((harq_process->K >> 3) - (harq_process->F >> 3) - ((harq_process->C > 1) ? 3 : 0));
  	}
  } else {
  	//set_abort(&harq_process->abort_decode, false);
  	for (int r = 0; r < harq_process->C; r++) {
    		int E = nr_get_E(G, harq_process->C, Qm, n_layers, r);
    		union ldpcReqUnion id = {.s = {ulsch->rnti, frame, nr_tti_rx, 0, A}};
    		//union ldpcReqUnion id = {.s = {0, 0, 0, 0, A}};
    		notifiedFIFO_elt_t *req = newNotifiedFIFO_elt(sizeof(ldpcDecode_t), id.s.spare, &phy_vars_gNB->respDecode, &nr_processULSegment);
    		ldpcDecode_t *rdata = (ldpcDecode_t *)NotifiedFifoData(req);
    		decParams.R = nr_get_R_ldpc_decoder(pusch_pdu->pusch_data.rv_index,
                                        E,
                                        decParams.BG,
                                        decParams.Z,
                                        &harq_process->llrLen,
                                        harq_process->round);
    		rdata->gNB = phy_vars_gNB;
    		rdata->ulsch_harq = harq_process;
    		rdata->decoderParms = decParams;
    		rdata->ulsch_llr = ulsch_llr;
    		rdata->Kc = decParams.BG == 2 ? 52 : 68;
    		rdata->harq_pid = harq_pid;
    		rdata->segment_r = r;
    		rdata->nbSegments = harq_process->C;
    		rdata->E = E;
    		rdata->A = A;
    		rdata->Qm = Qm;
    		rdata->r_offset = r_offset;
    		rdata->Kr_bytes = harq_process->K >> 3;
    		rdata->rv_index = pusch_pdu->pusch_data.rv_index;
    		rdata->offset = offset;
    		rdata->ulsch = ulsch;
    		rdata->ulsch_id = ULSCH_id;
            	phy_vars_gNB->nbDecode_oai++;
    		rdata->tbslbrm = pusch_pdu->maintenance_parms_v3.tbSizeLbrmBytes;
    		pushTpool(&phy_vars_gNB->threadPool, req);
    		LOG_D(PHY, "Added a block to decode, in pipe: %d\n", r);
    		r_offset += E;
    		offset += ((harq_process->K >> 3) - (harq_process->F >> 3) - ((harq_process->C > 1) ? 3 : 0));
  	}
  }

  return harq_process->C;
}
