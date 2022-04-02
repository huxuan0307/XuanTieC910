package Core.IFU

import chisel3._
import chisel3.util._
import Core.{Config}

trait CacheConfig extends Config {
  def AddrWidth = XLEN.W
  def pc_reset = PcStart.U(AddrWidth)
  def PC_WIDTH = 40
  def ICacheRead_WIDTH = 4
  def TotalSize = 64 //Kb //c910 parameter
  def Ways: Int = 2
  def LineSize = 64 // byte, 块大小
  def CacheDataBits = LineSize*8 //1字节是8比特
  def CacheCatNum  = 4 //Cache读取次数
  def CacheCatBits = LineSize * 8/CacheCatNum //128
  def Sets = TotalSize * 1024 / LineSize / Ways //512 sets
  //  def OffsetBits = log2Up(LineSize) //对应的是字节标号，= 6
  //  def IndexBits = log2Up(Sets) //=9
  //  def AddrHighBits = 64 - PC_WIDTH
  //  def TagBits = PC_WIDTH - IndexBits - OffsetBits // = 25, C910里pc_width=40, tagbits=28, 采用虚拟地址作为index
  def TagBits = 28
  def TagHighdex = 39
  def TagLowdex = 12
  def IndexPCBits = 16
  def IndexHighdex = 14//13
  def IndexLowdex = 6//5 //目前暂假定就是实地址Index
  def DIndexLowdex = 4//3
  //def IndexBits = 9//todo:目前Icache和dcache存在共用定义，注意区分
  def DIndexBits = 11
  def axiDataBits = 128 //C910里似乎128bits
  def ICacheRespBits = 128
  def ICachePredecBits = 32
  def RetTimes = CacheDataBits/axiDataBits //=4*128/128=4
  def cacheUseTabCnt = 32 //是计数器？
  def sram_width = 32 //2048*32 bit
  def Banks = 4
  def MSHRSize = 4
  def RamhelperBits = 64
  def RefillTimes = CacheCatBits/RamhelperBits //=128/64=2
  //  def addrBundle = new Bundle {
  //    val addrhigh   = UInt(AddrHighBits.W)
  //    val tag        = UInt(TagBits.W)
  //    val index      = UInt(IndexBits.W)
  //    val Offset = UInt(OffsetBits.W)
  //  }
  //  def tag(addr: IndexedSeq[UInt]): IndexedSeq[UInt] = {
  //    addr.map(_(TagHighdex,TagLowdex))
  //  }
  //  def index(addr:IndexedSeq[UInt]): IndexedSeq[UInt] = {
  //    addr.map(_(IndexHighdex,IndexLowdex))
  //  }
  def ftag(addr: UInt): UInt = {
    addr(TagHighdex,TagLowdex)
  }
  def findex(addr:UInt): UInt = {
    addr(IndexHighdex,IndexLowdex)
  }
  def fDindex(addr:UInt): UInt = {
    addr(IndexHighdex,DIndexLowdex)
  }
}

trait ifuConfig {
  def btb_index_bits = 10
  def btb_tag_bits = 10
  def btb_taget_pc_width = 20
  def l0_btb_update_entry_width = 16
  def l0_btb_wen_width = 4
  def pcgen_pc_width = 39
  def way_pred_width = 2
  def mmu_va_width = 63
}

class icache_RefillReq extends Bundle with CacheConfig {
  val paddr = Output(UInt(PC_WIDTH.W))
}

class icache_RefillResp extends Bundle with CacheConfig {
  val rdata = Output(UInt(CacheCatBits.W))
  val rvalid = Output(Bool())
}

class icache_RefillIO extends Bundle with CacheConfig{
  val req = Flipped(ValidIO(new icache_RefillReq))
  val resp = new icache_RefillResp
}

class cohResp extends Bundle with CacheConfig {
  val forward = Vec(CacheCatNum,UInt((LineSize/CacheCatNum*8).W))
  val needforward = Bool()
}