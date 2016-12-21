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
UDT允许用户访问两个拥塞控制参数：拥塞窗口大小和包与包之间的发送间隔。用户可以调整这两个参数来实现基于窗口的控制算法、
基于速率的控制算法或者是一套混合的控制算法。初次之外，以下参数也是用户可见的。
1）RTT
2）最大报文段/包大小
3）估计带宽
4）最近发送的最大报文序列号
5）包在接收端的到达速率

拥塞窗口的初始值为16，包发送间隔的初始值为0

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

// 设置ACK间隔； -1，则代表不启动这种ACK确认方法
void CCC::setACKInterval(int pktINT)
{
   m_iACKInterval = pktINT;
}

// 在UDT中，RTO=4*RTT+RTTVar
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

// 当UDT socket 建立连接时调用
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
收到ACK报文时：
1）如果当前在慢启动状态，拥塞窗口大小=包到达速率（注：接收端统计后，通过ACK捎带给发送端）*（RTT+SYN）。慢启动结束。停止。
2）如果不是在满启动状态，拥塞窗口大小（CWND）=A*（RTT+SYN）+16.
3）在下一个SYN period，需要增加发送的包inc为：
if （B <=C）
inc = min_inc;
else
inc = max（10^ceil（log10（10（（B-C）*PS*8）））*Beta/PS，min_inc）
其中，B是估计的链路容量，C是当前的发送速率，它们的单位均为packets/s。Beta是一个常量0.0000015，min_inc是一个最小增长值，
0.01-i.e，我们每秒钟至少增加一个包。（如何理解0.01代表每秒至少增加一个包？）
4）发送间隔更新为：SND=（SND*SYN）/（SND*inc+SYN）。以下四个参数用于速率减小的时候，它们的初始值为AvgNAKNum（1），NAKCount（1），DecCount（1），LastDecSeq（和初始的序列号一样为-1）
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

   m_LastRCTime = currtime;		//更新速率增长的时刻  

   if (m_bSlowStart)			//如果是在慢启动阶段 
   {
      m_dCWndSize += CSeqNo::seqlen(m_iLastAck, ack);	//增加发送窗口
      m_iLastAck = ack;

      if (m_dCWndSize > m_dMaxCWndSize)					//如果大于最大发送窗口 
      {
         m_bSlowStart = false;							//退出慢启动阶段  
         if (m_iRcvRate > 0)							//这个是ACK反馈回来的接收速率信息，赋值为m_iRcvRate	
            m_dPktSndPeriod = 1000000.0 / m_iRcvRate;	//m_dPktSndPeriod是包的发送间隔us
         else
            m_dPktSndPeriod = (m_iRTT + m_iRCInterval) / m_dCWndSize;
      }
   }
   else   //如果不在慢启动阶段 
      m_dCWndSize = m_iRcvRate / 1000000.0 * (m_iRTT + m_iRCInterval) + 16;
	  //由于m_iRcvRate单位是packet/s，而m_iRTT等是us。所以要对m_iRcvRate进行单位换算，发送窗口是通过接收速率*（RTT+RCInterval）+16来控制的

   // During Slow Start, no rate increase
   if (m_bSlowStart)
      return;

   if (m_bLoss)
   {
      m_bLoss = false;	//收到ACK代表没有丢失，改变状态
      return;
   }

   B = (int64_t)(m_iBandwidth - 1000000.0 / m_dPktSndPeriod);				//理论上是网络中空闲的带宽  
   if ((m_dPktSndPeriod > m_dLastDecPeriod) && ((m_iBandwidth / 9) < B))	//第二个表达式有什么实际意义，没想明白 
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
   /*分析下上面那个公式*/
   /*
   m_dPktSndPeriod = (m_dPktSndPeriod)*(m_iRCInterval/(m_dPktSndPeriod*inc + m_iRCInterval))),
   其中(m_iRCInterval/(m_dPktSndPeriod*inc + m_iRCInterval))<1,inc越大这个
   式子的值就越小，从而得到的m_dPktSndPeriod就越小，发送速率就越大，
   至于为什么要采取这样的计算方式就不知道了
   */
}

/**************************************************************************
收到NAK包时：
1）如果在慢启动阶段，结束慢启动。如果inter-packet interval=1/recvrate,则停止（注：代码中则调用return）。否则，根据当前的窗口大小（cwnd/rtt+SYN）
   设置发送速率（也就是发包间隔inter-packet interval），然后跳转到step2）继续减小发送速率；
2）如果这个NAK开启了一个新的拥塞时段，增加包的发送间隔inter-packet interval（snd）snd=snd*1.125；更新 AvgNAKNum，将NAKCount重置为1，
   计算DecRandom，从1和AvgNAKNum中产生随机数（平均分布）赋值给DecRandom。更新LastDecSeq，停止。
3）如果DecCount<=5并且NAKCount==DecCount*DecRandom：
a.更新SND时间：SND=SND*1.125；
b.将DecCount加1；
c.记录当前发送的最大序列号（LastDecSeq）
**************************************************************************/
void CUDTCC::onLoss(const int32_t* losslist, int)
{
   //Slow Start stopped, if it hasn't yet
   if (m_bSlowStart)
   {
      m_bSlowStart = false;
      if (m_iRcvRate > 0)	//单位是每秒钟多少个包packets/s,在处理ACK报文的时候设置，根据接收段的接收速度来设置发送端的m_iRcvRate，从而控制发送速度。
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

// 当定时器超时时调用
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
