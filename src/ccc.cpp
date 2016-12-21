/*****************************************************************************
Copyright (c) 2001 - 2011, The Board of Trustees of the University of Illinois.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

* Redistributions of source code must retain the above
  copyright notice, this list of conditions and the
  following disclaimer.

* Redistributions in binary form must reproduce the
  above copyright notice, this list of conditions
  and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the University of Illinois
  nor the names of its contributors may be used to
  endorse or promote products derived from this
  software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*****************************************************************************/

/*****************************************************************************
written by
   Yunhong Gu, last updated 02/21/2013
*****************************************************************************/
/*****************************************************************************
UDT�����û���������ӵ�����Ʋ�����ӵ�����ڴ�С�Ͱ����֮��ķ��ͼ�����û����Ե���������������ʵ�ֻ��ڴ��ڵĿ����㷨��
�������ʵĿ����㷨������һ�׻�ϵĿ����㷨������֮�⣬���²���Ҳ���û��ɼ��ġ�
1��RTT
2������Ķ�/����С
3�����ƴ���
4��������͵���������к�
5�����ڽ��ն˵ĵ�������

ӵ�����ڵĳ�ʼֵΪ16�������ͼ���ĳ�ʼֵΪ0

written by sammy
*****************************************************************************/


#include "core.h"
#include "ccc.h"
#include <cmath>
#include <cstring>

CCC::CCC():
m_iSYNInterval(CUDT::m_iSYNInterval),
m_dPktSndPeriod(1.0),
m_dCWndSize(16.0),
m_iBandwidth(),
m_dMaxCWndSize(),
m_iMSS(),
m_iSndCurrSeqNo(),
m_iRcvRate(),
m_iRTT(),
m_pcParam(NULL),
m_iPSize(0),
m_UDT(),
m_iACKPeriod(0),
m_iACKInterval(0),
m_bUserDefinedRTO(false),
m_iRTO(-1),
m_PerfInfo()
{
}

CCC::~CCC()
{
   delete [] m_pcParam;
}

void CCC::setACKTimer(int msINT)
{
   m_iACKPeriod = msINT > m_iSYNInterval ? m_iSYNInterval : msINT;
}

// ����ACK����� -1���������������ACKȷ�Ϸ���
void CCC::setACKInterval(int pktINT)
{
   m_iACKInterval = pktINT;
}

// ��UDT�У�RTO=4*RTT+RTTVar
void CCC::setRTO(int usRTO)
{
   m_bUserDefinedRTO = true;
   m_iRTO = usRTO;
}

void CCC::sendCustomMsg(CPacket& pkt) const
{
   CUDT* u = CUDT::getUDTHandle(m_UDT);

   if (NULL != u)
   {
      pkt.m_iID = u->m_PeerID;
      u->m_pSndQueue->sendto(u->m_pPeerAddr, pkt);
   }
}

const CPerfMon* CCC::getPerfInfo()
{
   try
   {
      CUDT* u = CUDT::getUDTHandle(m_UDT);
      if (NULL != u)
         u->sample(&m_PerfInfo, false);
   }
   catch (...)
   {
      return NULL;
   }

   return &m_PerfInfo;
}

void CCC::setMSS(int mss)
{
   m_iMSS = mss;
}

void CCC::setBandwidth(int bw)
{
   m_iBandwidth = bw;
}

void CCC::setSndCurrSeqNo(int32_t seqno)
{
   m_iSndCurrSeqNo = seqno;
}

void CCC::setRcvRate(int rcvrate)
{
   m_iRcvRate = rcvrate;
}

void CCC::setMaxCWndSize(int cwnd)
{
   m_dMaxCWndSize = cwnd;
}

void CCC::setRTT(int rtt)
{
   m_iRTT = rtt;
}

void CCC::setUserParam(const char* param, int size)
{
   delete [] m_pcParam;
   m_pcParam = new char[size];
   memcpy(m_pcParam, param, size);
   m_iPSize = size;
}

//
CUDTCC::CUDTCC():
m_iRCInterval(),
m_LastRCTime(),
m_bSlowStart(),
m_iLastAck(),
m_bLoss(),
m_iLastDecSeq(),
m_dLastDecPeriod(),
m_iNAKCount(),
m_iDecRandom(),
m_iAvgNAKNum(),
m_iDecCount()
{
}

// ��UDT socket ��������ʱ����
void CUDTCC::init()
{
   m_iRCInterval = m_iSYNInterval;
   m_LastRCTime = CTimer::getTime();
   setACKTimer(m_iRCInterval);

   m_bSlowStart = true;
   m_iLastAck = m_iSndCurrSeqNo;
   m_bLoss = false;
   m_iLastDecSeq = CSeqNo::decseq(m_iLastAck);
   m_dLastDecPeriod = 1;
   m_iAvgNAKNum = 0;
   m_iNAKCount = 0;
   m_iDecRandom = 1;

   m_dCWndSize = 16;
   m_dPktSndPeriod = 1;
}

/******************************************************************************************
�յ�ACK����ʱ��
1�������ǰ��������״̬��ӵ�����ڴ�С=���������ʣ�ע�����ն�ͳ�ƺ�ͨ��ACK�Ӵ������Ͷˣ�*��RTT+SYN����������������ֹͣ��
2�����������������״̬��ӵ�����ڴ�С��CWND��=A*��RTT+SYN��+16.
3������һ��SYN period����Ҫ���ӷ��͵İ�incΪ��
if ��B <=C��
inc = min_inc;
else
inc = max��10^ceil��log10��10����B-C��*PS*8������*Beta/PS��min_inc��
���У�B�ǹ��Ƶ���·������C�ǵ�ǰ�ķ������ʣ����ǵĵ�λ��Ϊpackets/s��Beta��һ������0.0000015��min_inc��һ����С����ֵ��
0.01-i.e������ÿ������������һ��������������0.01����ÿ����������һ��������
4�����ͼ������Ϊ��SND=��SND*SYN��/��SND*inc+SYN���������ĸ������������ʼ�С��ʱ�����ǵĳ�ʼֵΪAvgNAKNum��1����NAKCount��1����DecCount��1����LastDecSeq���ͳ�ʼ�����к�һ��Ϊ-1��
******************************************************************************************/
void CUDTCC::onACK(int32_t ack)
{
   int64_t B = 0;
   double inc = 0;
   // Note: 1/24/2012
   // The minimum increase parameter is increased from "1.0 / m_iMSS" to 0.01
   // because the original was too small and caused sending rate to stay at low level
   // for long time.
   const double min_inc = 0.01;

   uint64_t currtime = CTimer::getTime();
   if (currtime - m_LastRCTime < (uint64_t)m_iRCInterval)
      return;

   m_LastRCTime = currtime;		//��������������ʱ��  

   if (m_bSlowStart)			//��������������׶� 
   {
      m_dCWndSize += CSeqNo::seqlen(m_iLastAck, ack);	//���ӷ��ʹ���
      m_iLastAck = ack;

      if (m_dCWndSize > m_dMaxCWndSize)					//�����������ʹ��� 
      {
         m_bSlowStart = false;							//�˳��������׶�  
         if (m_iRcvRate > 0)							//�����ACK���������Ľ���������Ϣ����ֵΪm_iRcvRate	
            m_dPktSndPeriod = 1000000.0 / m_iRcvRate;	//m_dPktSndPeriod�ǰ��ķ��ͼ��us
         else
            m_dPktSndPeriod = (m_iRTT + m_iRCInterval) / m_dCWndSize;
      }
   }
   else   //��������������׶� 
      m_dCWndSize = m_iRcvRate / 1000000.0 * (m_iRTT + m_iRCInterval) + 16;
	  //����m_iRcvRate��λ��packet/s����m_iRTT����us������Ҫ��m_iRcvRate���е�λ���㣬���ʹ�����ͨ����������*��RTT+RCInterval��+16�����Ƶ�

   // During Slow Start, no rate increase
   if (m_bSlowStart)
      return;

   if (m_bLoss)
   {
      m_bLoss = false;	//�յ�ACK����û�ж�ʧ���ı�״̬
      return;
   }

   B = (int64_t)(m_iBandwidth - 1000000.0 / m_dPktSndPeriod);				//�������������п��еĴ���  
   if ((m_dPktSndPeriod > m_dLastDecPeriod) && ((m_iBandwidth / 9) < B))	//�ڶ������ʽ��ʲôʵ�����壬û������ 
      B = m_iBandwidth / 9;
   if (B <= 0)
      inc = min_inc;
   else
   {
      // inc = max(10 ^ ceil(log10( B * MSS * 8 ) * Beta / MSS, 1/MSS)
      // Beta = 1.5 * 10^(-6)

      inc = pow(10.0, ceil(log10(B * m_iMSS * 8.0))) * 0.0000015 / m_iMSS;

      if (inc < min_inc)
         inc = min_inc;
   }

   m_dPktSndPeriod = (m_dPktSndPeriod * m_iRCInterval) / (m_dPktSndPeriod * inc + m_iRCInterval);
   /*�����������Ǹ���ʽ*/
   /*
   m_dPktSndPeriod = (m_dPktSndPeriod)*(m_iRCInterval/(m_dPktSndPeriod*inc + m_iRCInterval))),
   ����(m_iRCInterval/(m_dPktSndPeriod*inc + m_iRCInterval))<1,incԽ�����
   ʽ�ӵ�ֵ��ԽС���Ӷ��õ���m_dPktSndPeriod��ԽС���������ʾ�Խ��
   ����ΪʲôҪ��ȡ�����ļ��㷽ʽ�Ͳ�֪����
   */
}

/**************************************************************************
�յ�NAK��ʱ��
1��������������׶Σ����������������inter-packet interval=1/recvrate,��ֹͣ��ע�������������return�������򣬸��ݵ�ǰ�Ĵ��ڴ�С��cwnd/rtt+SYN��
   ���÷������ʣ�Ҳ���Ƿ������inter-packet interval����Ȼ����ת��step2��������С�������ʣ�
2��������NAK������һ���µ�ӵ��ʱ�Σ����Ӱ��ķ��ͼ��inter-packet interval��snd��snd=snd*1.125������ AvgNAKNum����NAKCount����Ϊ1��
   ����DecRandom����1��AvgNAKNum�в����������ƽ���ֲ�����ֵ��DecRandom������LastDecSeq��ֹͣ��
3�����DecCount<=5����NAKCount==DecCount*DecRandom��
a.����SNDʱ�䣺SND=SND*1.125��
b.��DecCount��1��
c.��¼��ǰ���͵�������кţ�LastDecSeq��
**************************************************************************/
void CUDTCC::onLoss(const int32_t* losslist, int)
{
   //Slow Start stopped, if it hasn't yet
   if (m_bSlowStart)
   {
      m_bSlowStart = false;
      if (m_iRcvRate > 0)	//��λ��ÿ���Ӷ��ٸ���packets/s,�ڴ���ACK���ĵ�ʱ�����ã����ݽ��նεĽ����ٶ������÷��Ͷ˵�m_iRcvRate���Ӷ����Ʒ����ٶȡ�
      {
         // Set the sending rate to the receiving rate.
         m_dPktSndPeriod = 1000000.0 / m_iRcvRate;
         return;
      }
      // If no receiving rate is observed, we have to compute the sending
      // rate according to the current window size, and decrease it
      // using the method below.
      m_dPktSndPeriod = m_dCWndSize / (m_iRTT + m_iRCInterval);
   }

   m_bLoss = true;

   if (CSeqNo::seqcmp(losslist[0] & 0x7FFFFFFF, m_iLastDecSeq) > 0)
   {
      m_dLastDecPeriod = m_dPktSndPeriod;
      m_dPktSndPeriod = ceil(m_dPktSndPeriod * 1.125);

      m_iAvgNAKNum = (int)ceil(m_iAvgNAKNum * 0.875 + m_iNAKCount * 0.125);
      m_iNAKCount = 1;
      m_iDecCount = 1;

      m_iLastDecSeq = m_iSndCurrSeqNo;

      // remove global synchronization using randomization
      srand(m_iLastDecSeq);
      m_iDecRandom = (int)ceil(m_iAvgNAKNum * (double(rand()) / RAND_MAX));
      if (m_iDecRandom < 1)
         m_iDecRandom = 1;
   }
   else if ((m_iDecCount ++ < 5) && (0 == (++ m_iNAKCount % m_iDecRandom)))
   {
      // 0.875^5 = 0.51, rate should not be decreased by more than half within a congestion period
      m_dPktSndPeriod = ceil(m_dPktSndPeriod * 1.125);
      m_iLastDecSeq = m_iSndCurrSeqNo;
   }
}

// ����ʱ����ʱʱ����
void CUDTCC::onTimeout()
{
   if (m_bSlowStart)
   {
      m_bSlowStart = false;
      if (m_iRcvRate > 0)
         m_dPktSndPeriod = 1000000.0 / m_iRcvRate;
      else
         m_dPktSndPeriod = m_dCWndSize / (m_iRTT + m_iRCInterval);
   }
   else
   {
      /*
      m_dLastDecPeriod = m_dPktSndPeriod;
      m_dPktSndPeriod = ceil(m_dPktSndPeriod * 2);
      m_iLastDecSeq = m_iLastAck;
      */
   }
}
