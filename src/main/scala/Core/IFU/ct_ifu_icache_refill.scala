package Core.IFU

import Core.Config
import chisel3._
import chisel3.util._


//
class ct_ifu_icache_refill extends Module with Config with CacheConfig {
  val io = IO(new icache_RefillIO)

  val paddr = Cat(0.U((XLEN-PAddrBits).W),io.req.bits.paddr) - PcStart.U(XLEN.W)//todo: change addr width
  val paddrReg = RegEnable(paddr,0.U(XLEN.W),io.req.valid)





  val ram = Module(new RAMHelper)
  ram.io.clk := clock
  ram.io.en := !reset.asBool()
  ram.io.rIdx := DontCare
  ram.io.wIdx := DontCare
  ram.io.wen := false.B
  ram.io.wdata := DontCare
  ram.io.wmask := DontCare


  val s_pf0 :: s_pf0_done :: s_pf1 :: s_pf1_done :: s_pf2 :: s_pf2_done :: s_pf3 :: s_pf3_done :: Nil = Enum(8)
  val state: UInt = RegInit(s_pf0)
  //replace等待axi4响应，如果是ramhelper一次64bit数据，一个缓存块64byte需要8次传输

  val SRam_write  =RegInit(VecInit(Seq.fill(RefillTimes)(0.U(RamhelperBits.W)))) //往法


  //L2测试可先用ramhelper
  when(state === s_pf0 && io.req.valid) {
    ram.io.rIdx := (paddr ) >> 3.U//64bit一跳转//第一拍直接以请求当拍的paddr读，第二拍开始是用寄存器地址
    SRam_write(0)  := ram.io.rdata
  }.elsewhen(state === s_pf0_done){
    ram.io.rIdx := (paddrReg + 8.U) >> 3.U//64bit一跳转 //pc_reset(PC_WIDTH - 1, 0)
    SRam_write(1)  := ram.io.rdata
  }.elsewhen(state === s_pf1){
    ram.io.rIdx := (paddrReg + 16.U) >> 3.U
    SRam_write(0)  := ram.io.rdata
  }.elsewhen(state === s_pf1_done){
    ram.io.rIdx := (paddrReg + 24.U) >> 3.U
    SRam_write(1)  := ram.io.rdata
  }.elsewhen(state === s_pf2){
    ram.io.rIdx := (paddrReg + 32.U) >> 3.U
    SRam_write(0)  := ram.io.rdata
  }.elsewhen(state === s_pf2_done){
    ram.io.rIdx := (paddrReg + 40.U) >> 3.U
    SRam_write(1)  := ram.io.rdata
  }.elsewhen(state === s_pf3){
    ram.io.rIdx := (paddrReg + 48.U) >> 3.U
    SRam_write(0)  := ram.io.rdata
  }.elsewhen(state === s_pf3_done){
    ram.io.rIdx := (paddrReg + 56.U) >> 3.U
    SRam_write(1)  := ram.io.rdata
  }

  io.resp.rdata := Cat(SRam_write(1),SRam_write(0))
  io.resp.rvalid :=
    (RegNext(state === s_pf0_done)) ||
    (RegNext(state === s_pf1_done)) ||
    (RegNext(state === s_pf2_done)) ||
    (RegNext(state === s_pf3_done))

  //-------------------------------------状态机------------------------------------------------
  switch(state) {
    is(s_pf0) {
      when(io.req.valid) {
        state := s_pf0_done
      }
    }
    is(s_pf0_done) {
      state := s_pf1
    }
    is(s_pf1) {
      state := s_pf1_done
    }
    is(s_pf1_done) {
      state := s_pf2
    }
    is(s_pf2) {
      state := s_pf2_done
    }
    is(s_pf2_done) {
      state := s_pf3
    }
    is(s_pf3) {
      state := s_pf3_done
    }
    is(s_pf3_done) {
      state := s_pf0
    }
  }
}
