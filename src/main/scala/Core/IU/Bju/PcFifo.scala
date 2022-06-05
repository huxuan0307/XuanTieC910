package Core.IU.Bju

import Core.AddrConfig.PcWidth
import Core.Config.PcStart
import Core.IUConfig.{PcFifoAddr, PcFifoLen}
import Core.IntConfig.InstructionIdWidth
import Core.ROBConfig.NumRobReadEntry
import Core.RTU.PcFifoData
import Utils.{CircularQueuePtr, HasCircularQueuePtrHelper}
import chisel3._
import chisel3.util._

class PcFifoPtr extends CircularQueuePtr[PcFifoPtr](PcFifoLen){

}

object PcFifoPtr {
  def apply(f: Bool, v:UInt): PcFifoPtr = {
    val ptr = Wire(new PcFifoPtr)
    ptr.flag := f
    ptr.value := v
    ptr
  }
}

class IduWriteIO extends Bundle{
  val writePcfifo  = Vec(2, Input(new ifuDataForward))
  val isIn         = Input(new IsuAlloFifo)
  val alloPid      = Vec(4, Output(UInt(PcFifoAddr.W)))
  val isFull       = Output(Bool())
}
class BjuRwPidIO extends Bundle{
  val rfPipe2 = Input(UInt(PcFifoAddr.W)) // pipe 2 pid
  val ex2     = Input(UInt(PcFifoAddr.W))
  val ex3     = Input(UInt(PcFifoAddr.W))
}
class BjuEx3Bypss extends Bundle{
    val instVld      = Bool()
    val iid          = UInt(InstructionIdWidth.W)
    val bht_mispred  = Bool()
    val pc           = UInt(PcWidth.W)
    val bht_pred     = Bool()
    val pid          = UInt(PcFifoAddr.W)
    val conbr        = Bool()
    val jmp          = Bool()
    val pret         = Bool()
    val pcall        = Bool()
    val length       = Bool()
}

class BjuRwIO extends Bundle {
  val readPcfifo  = Output(new IfuPredStore) //bju ex1 read pcfifo stored ifu data
  val pid         = new BjuRwPidIO
  val ex2WritePcfifo = Input((new PredCheckRes)) //ex2 write checked ifu pred data
  val ex3Bypass = Input(new BjuEx3Bypss)
  val specialPid     = Input(UInt(PcFifoAddr.W)) // pipe 0 pid, actually from cp0
  val specialPc      = Output(UInt(PcWidth.W))
  val iuYyXxCancel = Input(Bool())
}
class RtuReadIO extends Bundle{
  val rtuReadData = Output(Vec(NumRobReadEntry,new PcFifoData))
  val readPcfifovld = Input(Vec(NumRobReadEntry,Bool()))
  val rtuFlush = Input(new Bundle() {
    val fe = (Bool())
    val flush = Bool()
  })
}
class PcFifoIO extends Bundle{
  val iduWrite = new IduWriteIO // valid to out - pcfifo is full ? bht can not in : bht can in
  val bjuRw    = new BjuRwIO
  val rtuRw    = new RtuReadIO
}

class PcFifo extends Module with HasCircularQueuePtrHelper{
  // PcFifo storage 3 data: chk_idx, bht_pred, jmp_mispred
  val io = IO(new PcFifoIO())
  val createPtr = RegInit(PcFifoPtr(false.B, 0.U))
//  val tailPtr = RegInit(PcFifoPtr(true.B,  0.U))
  val pop_ptr = RegInit(PcFifoPtr(false.B, 0.U))
  val is_full = isFull((createPtr+3.U),pop_ptr)//distanceBetween(tailPtr, createPtr) < 3.U
  val fifo_create_vld  = VecInit(Seq.fill(2)(RegInit(false.B)))
  val ifu_forward_data = VecInit(Seq.fill(2)(RegInit(0.U.asTypeOf((new ifuDataForward)))))
  for(i <- 0 until 2){
    fifo_create_vld(i) := Mux(io.rtuRw.rtuFlush.fe || io.bjuRw.iuYyXxCancel,false.B,io.iduWrite.writePcfifo(i).en)
    when(!is_full && io.iduWrite.writePcfifo(i).en){
      ifu_forward_data(i) := io.iduWrite.writePcfifo(i)
    }
  }
  io.iduWrite.isFull := is_full

  // define 2 table
  var fifo_tab_pred  = RegInit(VecInit(Seq.fill(PcFifoLen)( 0.U.asTypeOf(new IfuPredStore) ))) // ifu pred data tab
  var fifo_tab_check = RegInit(VecInit(Seq.fill(PcFifoLen)( 0.U.asTypeOf(new PredCheckRes) ))) // bju jmp&br check tab
  //----------------------------------------------------------
  //               idu write in store & allocate ptr
  //----------------------------------------------------------
  //                       allocate ptr
  //                   j take 2 & br take 1
  //----------------------------------------------------------
  val fifo_create_vec = VecInit(Seq.fill(3)(WireInit(false.B)))
  //********************************************************
  // vec priority  2 > 1 > 0
  // case 0 : shift 1 bit
  //    (0,1) = (T,X) : 0 is vld
  // case 1 : shift 2 bits
  //    (0,1) = (T,T) : 2 br inst
  //    (0,1) = (T,F) : 1 jr inst
  // case 2 : shift 3 bits
  //    (0,1) = (T,T) : 1 br and 1 jr
  // ###### will not appear (F,T) situation
  //********************************************************
  fifo_create_vec(0) := fifo_create_vld(0) && !io.bjuRw.iuYyXxCancel && !io.rtuRw.rtuFlush.fe
  fifo_create_vec(1) := ((fifo_create_vld(0) && ifu_forward_data(0).dstVld ) || fifo_create_vld(1))  && !io.bjuRw.iuYyXxCancel && !io.rtuRw.rtuFlush.fe
  fifo_create_vec(2) := (fifo_create_vld.reduce(_ && _) && (ifu_forward_data(0).dstVld || ifu_forward_data(1).dstVld)) && !io.bjuRw.iuYyXxCancel && !io.rtuRw.rtuFlush.fe
  //--------------------------------------------------------
  // store data generate
  val fifo_create_data = VecInit(Seq.fill(3)(WireInit(0.U.asTypeOf(new IfuPredStore))))
  fifo_create_data(0).pc          := Mux(ifu_forward_data(0).jalr&& (!ifu_forward_data(0).dstVld),  ifu_forward_data(0).tarPc, ifu_forward_data(0).curPc)
  fifo_create_data(0).chkIdx      := ifu_forward_data(0).predStore.chkIdx
  fifo_create_data(0).bhtPred     := ifu_forward_data(0).predStore.bhtPred
  fifo_create_data(0).jmpMispred  := ifu_forward_data(0).predStore.jmpMispred

  fifo_create_data(1).pc          := Mux(ifu_forward_data(0).dstVld,
    Mux(ifu_forward_data(0).jal, ifu_forward_data(0).curPc ,ifu_forward_data(0).tarPc ),
    Mux(ifu_forward_data(1).jalr,ifu_forward_data(1).curPc ,ifu_forward_data(1).tarPc ))
  fifo_create_data(1).chkIdx      := Mux(ifu_forward_data(0).dstVld, ifu_forward_data(0).predStore.chkIdx    , ifu_forward_data(1).predStore.chkIdx )
  fifo_create_data(1).bhtPred     := Mux(ifu_forward_data(0).dstVld, ifu_forward_data(0).predStore.bhtPred   , ifu_forward_data(1).predStore.bhtPred )
  fifo_create_data(1).jmpMispred  := Mux(ifu_forward_data(0).dstVld, ifu_forward_data(0).predStore.jmpMispred, ifu_forward_data(1).predStore.jmpMispred )

  fifo_create_data(2).pc         := Mux(ifu_forward_data(1).jalr,  ifu_forward_data(1).tarPc, ifu_forward_data(1).curPc)
  fifo_create_data(2).chkIdx     := ifu_forward_data(1).predStore.chkIdx
  fifo_create_data(2).bhtPred    := ifu_forward_data(1).predStore.bhtPred
  fifo_create_data(2).jmpMispred := ifu_forward_data(1).predStore.jmpMispred
  // ptr create
  val create_ptr_num = PopCount(fifo_create_vec)

  createPtr := createPtr + create_ptr_num
//  // fifo ctrl signal
//  val ptr_vld = RegInit(false.B)
//  val ptr_flush = RegInit(false.B)
//  val ptr_cmplt = RegInit(false.B)
//  when(ptr_vld && io.rtuRw.rtuFlush.flush && (ptr_flush || io.bjuRw.iuYyXxCancel || io.rtuRw.rtuFlush.fe)){
//    ptr_vld := false.B
//  }.elsewhen(create_en){
//    ptr_vld := true.B
//  }.elsewhen(pop_en){
//    ptr_vld := false.B
//  }
//  when(ptr_vld && io.rtuRw.rtuFlush.flush && (ptr_flush || io.bjuRw.iuYyXxCancel || io.rtuRw.rtuFlush.fe)){
//    ptr_cmplt := false.B
//  }.elsewhen(create_en){
//    ptr_cmplt := false.B
//  }.elsewhen(cmplt_en){
//    ptr_cmplt := true.B
//  }.elsewhen(pop_en){
//    ptr_cmplt := false.B
//  }
//  when(ptr_vld && (io.bjuRw.iuYyXxCancel || io.rtuRw.rtuFlush.fe) && !io.rtuRw.rtuFlush.flush){
//    ptr_flush := true.B
//  }.elsewhen(create_en){
//    ptr_flush := false.B
//  }
//
  // ptr to idu
  val assign_ptr = RegInit(PcFifoPtr(false.B, 0.U))
  val to_idu_pid_vec = WireInit(VecInit(Seq.tabulate(4)(i => (i).U(PcFifoAddr.W))))
  when(io.bjuRw.iuYyXxCancel || io.rtuRw.rtuFlush.fe){
    assign_ptr := createPtr
  }.elsewhen(io.iduWrite.isIn.instVld){
    assign_ptr := assign_ptr + io.iduWrite.isIn.instNum
  }// .otherwise assign_ptr:= assign_ptr
  for(i<- 0 until 4){
    val ptr = assign_ptr.value
    to_idu_pid_vec(i) := assign_ptr.value + i.U
  }
  io.iduWrite.alloPid := to_idu_pid_vec
  // pop ptr
  val pop_en = io.rtuRw.readPcfifovld.reduce(_ || _)
  val pop_num = PopCount(io.rtuRw.readPcfifovld)

  when(io.rtuRw.rtuFlush.flush && (io.bjuRw.iuYyXxCancel || io.rtuRw.rtuFlush.fe)){
    pop_ptr := createPtr
  }.elsewhen(io.rtuRw.rtuFlush.flush){
    pop_ptr := assign_ptr
  }.elsewhen(pop_en){
    pop_ptr := pop_ptr + pop_num
  }// .otherwise pop_ptr:= pop_ptr
  //----------------------------------------------------------
  //                   bju read by pid
  //----------------------------------------------------------
  val bju_read_pid = io.bjuRw.pid.rfPipe2
  val readData = fifo_tab_pred(bju_read_pid) // VecInit(bju_read_pid.map(addr => fifo_tab_pred(addr)))
  io.bjuRw.readPcfifo := readData
  //----------------------------------------------------------
  //                  pcfifo write
  //----------------------------------------------------------
  val create_en = create_ptr_num =/= 0.U
  val cmplt_en = io.bjuRw.ex2WritePcfifo.instVld
  // idu write
  for(i <- 0 until 3){
    //      val offset = Mux(fifo_create_vec(i), 1.U, 0.U) // shift once per vec
    val ptr = createPtr + i.U
    val idx = ptr.value
    when(i.U <= create_ptr_num){
      fifo_tab_pred(idx) := fifo_create_data(i)
    }.otherwise{
      fifo_tab_pred(idx) := fifo_tab_pred(idx)
    }
  }
  // bju write
  val bju_write_pid = io.bjuRw.pid.ex2
  when(create_en){
    fifo_tab_check(bju_write_pid) := 0.U.asTypeOf(new PredCheckRes)
  }.elsewhen(cmplt_en){
    fifo_tab_check(bju_write_pid) := io.bjuRw.ex2WritePcfifo
  }

  //----------------------------------------------------------
  //              rob read & deallocate ptr
  //----------------------------------------------------------
  val ex2_bypass_data = WireInit(0.U.asTypeOf(new PcFifoData))
  ex2_bypass_data.length       := io.bjuRw.ex2WritePcfifo.length
  ex2_bypass_data.bhtPred      := io.bjuRw.ex2WritePcfifo.bhtPred
  ex2_bypass_data.bhtMispred   := io.bjuRw.ex2WritePcfifo.bhtMispred
  ex2_bypass_data.jmp          := io.bjuRw.ex2WritePcfifo.jmp
  ex2_bypass_data.pret         := io.bjuRw.ex2WritePcfifo.pret
  ex2_bypass_data.pcall        := io.bjuRw.ex2WritePcfifo.pcall
  ex2_bypass_data.condBranch   := io.bjuRw.ex2WritePcfifo.condbr
  ex2_bypass_data.pcNext       := io.bjuRw.ex2WritePcfifo.pc
  ex2_bypass_data.lsb := DontCare
  val ex3_bypass_data = WireInit(0.U.asTypeOf(new PcFifoData))
  ex3_bypass_data.length       := io.bjuRw.ex3Bypass.length
  ex3_bypass_data.bhtPred      := io.bjuRw.ex3Bypass.bht_pred
  ex3_bypass_data.bhtMispred   := io.bjuRw.ex3Bypass.bht_mispred
  ex3_bypass_data.jmp          := io.bjuRw.ex3Bypass.jmp
  ex3_bypass_data.pret         := io.bjuRw.ex3Bypass.pret
  ex3_bypass_data.pcall        := io.bjuRw.ex3Bypass.pcall
  ex3_bypass_data.condBranch   := io.bjuRw.ex3Bypass.conbr
  ex3_bypass_data.pcNext       := io.bjuRw.ex3Bypass.pc
  ex3_bypass_data.lsb := DontCare
  val pop_ptr_vec  = VecInit(Seq.fill(3)(WireInit(0.U(PcFifoAddr.W))))
  val fifo_pop_data = VecInit(Seq.fill(NumRobReadEntry)(WireInit(0.U.asTypeOf(new PcFifoData))))
  for (i<-0 until(3)){
    pop_ptr_vec(i) := pop_ptr.value + pop_num + i.U
  }
  for (i<-0 until(3)){
    fifo_pop_data(i).length      := fifo_tab_check(pop_ptr_vec(i)).length
    fifo_pop_data(i).bhtPred     := fifo_tab_pred(pop_ptr_vec(i)).bhtPred
    fifo_pop_data(i).bhtMispred  := fifo_tab_check(pop_ptr_vec(i)).bhtMispred
    fifo_pop_data(i).jmp         := fifo_tab_check(pop_ptr_vec(i)).jmp
    fifo_pop_data(i).pret        := fifo_tab_check(pop_ptr_vec(i)).pret
    fifo_pop_data(i).pcall       := fifo_tab_check(pop_ptr_vec(i)).pcall
    fifo_pop_data(i).condBranch  := fifo_tab_check(pop_ptr_vec(i)).condbr
    fifo_pop_data(i).pcNext      := Mux(create_en,fifo_tab_pred(pop_ptr_vec(i)).pc ,fifo_tab_check(pop_ptr_vec(i)).pc)
  }

  val pop_data_vec = VecInit(Seq.fill(NumRobReadEntry)(WireInit(0.U.asTypeOf(new PcFifoData))))
  for (i<-0 until(3)){
    pop_data_vec(i) := Mux((pop_ptr.value===io.bjuRw.ex3Bypass.pid)&&io.bjuRw.ex3Bypass.instVld,ex3_bypass_data,
      Mux((pop_ptr.value=== io.bjuRw.pid.ex2)&& io.bjuRw.ex2WritePcfifo.instVld,ex2_bypass_data, fifo_pop_data(i))
    )
  }

  io.rtuRw.rtuReadData := pop_data_vec
  for(i <- 0 until NumRobReadEntry){
    io.rtuRw.rtuReadData(i).pcNext := (pop_data_vec(i).pcNext - PcStart.U) >> 1.U
  }

  //----------------------------------------------------------
  //         Speical read, to Special Unit, auipc inst
  //----------------------------------------------------------
  io.bjuRw.specialPc  :=  fifo_tab_pred(io.bjuRw.specialPid).pc
}
