package Core.IU.Csr

import Core.Define.Exceptions.ExceptionVec
import Core.TrapIO
import Core.Define.Exceptions._
import chisel3._
import chisel3.internal.firrtl.Width
import chisel3.util._
import Privilege.{supportSupervisor, supportUser}
import chisel3.util.experimental.BoringUtils

import scala.math.pow
// hart error process order
// err pc store in mepc, pc setted as mtvec
// set mcause, and set mtval as err addr or err Word
// set mstatus - MIE as 0 to ban the interrupt, and save MIE to MPIE
// save current priv mode to mstatus - MPP, then set priv to M-mode
// mscratch point to the temp pointer which point to additional temp memory, the mem save the integ regs data

// start exe main

// reverse the process, restore the site by mscratch
// return pc by mret instruction
// mret will: set pc , set mstatus, set mode


// S-mode, TLB, base sv39
// sv 39
//  |63 54|53 28|27 19|18 10|9 8|7|6|5|4|3|2|1|0|
//  +-----+-----+-----+-----+---+-+-+-+-+-+-+-+-+
//  |resev| PPN2| PPN1| PPN0|RSW|D|A|G|U|X|W|R|V|
//  V位决定了该页表项的其余部分是否有效（ V = 1时有效）。若 V = 0，则任何遍历
//  到此页表项的虚址转换操作都会导致页错误。
//  ⚫ R、 W和 X位分别表示此页是否可以读取、写入和执行。如果这三个位都是 0
//  那么这个页表项是指向下一级页表的指针，否则它是页表树 的一个叶节点。
//  ⚫ U位表示该页是否是用户页面。若 U = 0，则 U模式不能访问此页面，但 S模式
//  可以。若 U = 1，则 U模式下能访问这个页面，而 S模式不能。
//  ⚫ G位表示这个映射是否对所有虚址空间有效，硬件可以用这个信息来提高地址转
//  换的性能。这一位通常只用于属于操作系统的页面。
//  ⚫ A位表示自从上次 A位被清除以来，该页面是否被访问过。
//  ⚫ D位表示自从上次清除 D位以来页面是否被弄脏（例如被写入）。
//  ⚫ RSW域留给操作系统使用，它会被硬件忽略。
//  ⚫ PPN域包含物理页号，这是物理地址的一部分。若这个页表项是一个叶节点，那
//  么 PPN是转换后物理地址的一部分。否则 PPN给出下一节页表的地址。


object FuncOpType {
  def width = 7.W
  def uwidth = UInt(width)
}

object CsrOpType {
  def RW        : UInt = "b000001".U(FuncOpType.width)
  def RS        : UInt = "b000010".U(FuncOpType.width)
  def RC        : UInt = "b000011".U(FuncOpType.width)
  def RWI       : UInt = "b000101".U(FuncOpType.width)
  def RSI       : UInt = "b000110".U(FuncOpType.width)
  def RCI       : UInt = "b000111".U(FuncOpType.width)
  def ECALL     : UInt = "b010000".U(FuncOpType.width)
  def EBREAK    : UInt = "b010001".U(FuncOpType.width)
  def MRET      : UInt = "b011000".U(FuncOpType.width)
  def SRET      : UInt = "b011001".U(FuncOpType.width)
  def URET      : UInt = "b011011".U(FuncOpType.width)
  def INTERRUPT : UInt = "b100000".U(FuncOpType.width)
  def isJmp(op: UInt)       : Bool = op(4).asBool()
  def isRet(op: UInt)       : Bool = op(3).asBool()
  def isCsri(op: UInt)      : Bool = op(2).asBool()
  def isCall(op: UInt)      : Bool = op === ECALL
  def isInterrupt(op :UInt) : Bool = op(5).asBool()
}


trait Config{
  def XLEN = 64
  def SV39Len = 39
  def SXLEN : Int = MXLEN
  def UXLEN : Int = SXLEN
  // CSR
  // Machine

  // set MXL to decide the MXELN
  // 1 - 32, 2 - 64, 3 - 128
  def MXL = 2
  def MXLEN : Int = pow(2, MXL + 4).toInt

  def CSR_ADDR_LEN = 12

  def VendorID = 0

  def ArchitectureID = 0

  def ImplementationID = 0

  def HardwareThreadID = 0

  //  def getMisaExt(ext: List[Char]): Int = {
  //    var misaExt = 0
  //    for (i <- 0 until ext.length) {
  //      misaExt |= (ext(i).toInt - 'a'.toInt)
  //    }
  //    misaExt
  //  }
  // def ISAEXT = List('i')
  class ISAExt(string : String) {
    val value : Int = toInt(string)
    def support(ch : Char) : Boolean = {
      (value & ch.toInt) != 0
    }
    private def toInt(string : String) : Int = {
      var value = 0
      for (ch <- string) {
        require(ch.isLetter)
        value |= (ch.toUpper - 'A')
      }
      value
    }

  def toInt : Int = value
}
  def ISAEXT = new ISAExt("I")

  def MISA : BigInt = {
    BigInt(MXL) << (MXLEN - 2) | BigInt(ISAEXT.toInt)
  }
}

class CSR extends Module with Config with CsrRegDefine {// TODO: IO need to define
  class CSRIO extends Bundle with Config{
    val in         = Flipped(Valid(new FuInPut))
    val out       : Valid[FuOutPut] = Valid(new FuOutPut)
    val jmp       : Valid[RedirectIO]     = Valid(new RedirectIO)
    val bpu_update = ValidIO(new BPU_Update)
    val trapvalid  = Output(Bool())
    val skip       = Output(Bool())
  }
  val io = IO(new CSRIO())
  // working mode define
  private val mode_u::mode_s::mode_h::mode_m::Nil = Enum(4)
  private val currentPriv = RegInit(UInt(2.W), mode_m)
  // input decode data prepare
  private val (ena, src, csrAddr, op) = (
    io.in.valid,
    io.in.bits.src(0),
    io.in.bits.uop.data.imm(CSR_ADDR_LEN - 1, 0),
    io.in.bits.uop.ctrl.funcOpType
  )
  // read Core.csr regs
  private val rdata = MuxLookup(csrAddr, 0.U(MXLEN.W), readOnlyMap++readWriteMap)
  // write Core.csr regs, may need to cover the old value
  private val wdata = MuxLookup(op, 0.U, Array(
    CsrOpType.RW  ->  src,                        // read & write
    CsrOpType.RWI ->  src,
    CsrOpType.RS  ->  (rdata | src),              // read & set
    CsrOpType.RSI ->  (rdata | src),
    CsrOpType.RC  ->  (rdata & (~src).asUInt()),  // read & clean
    CsrOpType.RCI ->  (rdata & (~src).asUInt())
  ))
  private val is_jmp    : Bool = CsrOpType.isJmp(op)
  private val is_ret    : Bool = CsrOpType.isRet(op)     && is_jmp
  private val is_mret   : Bool = CsrOpType.MRET === op   && is_jmp
  private val is_sret   : Bool = CsrOpType.SRET === op   && is_jmp
  private val is_uret   : Bool = CsrOpType.URET === op   && is_jmp
  private val is_ebreak : Bool = CsrOpType.EBREAK === op && is_jmp
  private val is_ecall  : Bool = CsrOpType.isCall(op)

  private val trap_valid    = ena && is_jmp
  private val mstatus_en    = ena && !is_jmp && (csrAddr === CsrAddr.mstatus)
  private val medeleg_en    = ena && !is_jmp && (csrAddr === CsrAddr.medeleg)
  private val mideleg_en    = ena && !is_jmp && (csrAddr === CsrAddr.mideleg)
  private val mie_en        = ena && !is_jmp && (csrAddr === CsrAddr.mie)
  private val mtvec_en      = ena && !is_jmp && (csrAddr === CsrAddr.mtvec)
  private val mcounteren_en = ena && !is_jmp && (csrAddr === CsrAddr.mcounteren)
  private val mscratch_en   = ena && !is_jmp && (csrAddr === CsrAddr.mscratch)
  private val mepc_en       = ena && !is_jmp && (csrAddr === CsrAddr.mepc)
  private val mcause_en     = ena && !is_jmp && (csrAddr === CsrAddr.mcause)
  private val mtval_en      = ena && !is_jmp && (csrAddr === CsrAddr.mtval)
  private val mcycle_en     = ena && !is_jmp && (csrAddr === CsrAddr.mcycle)
  private val minstret_en   = ena && !is_jmp && (csrAddr === CsrAddr.minstret)

  when(mstatus_en){
    val mstatus_new = WireInit(wdata.asTypeOf(new StatusStruct))
    // TODO 分别把各特权级允许写的字段一一连线
    if (supportUser) {
    status.MPRV  :=  mstatus_new.MPRV
    status.MPP   :=  legalizePrivilege(mstatus_new.MPP)
    }
    if (supportSupervisor){



    }
    status.IE     := mstatus_new.IE
    status.PIE    := mstatus_new.PIE
    status.FS     := mstatus_new.FS
  }
  medeleg   := Mux(medeleg_en   , wdata, medeleg)
  mideleg   := Mux(mideleg_en   , wdata, mideleg)
  ie        := Mux(mie_en       , wdata.asTypeOf(new InterruptField), mie)
  mtvec     := Mux(mtvec_en     , wdata, mtvec)
  mcounteren:= Mux(mcounteren_en, wdata, mcounteren)
  mscratch  := Mux(mscratch_en  , wdata, mscratch)
  mepc      := Mux(mepc_en      , wdata, mepc)
  mcause    := Mux(mcause_en    , wdata, mcause)
  mtval     := Mux(mtval_en     , wdata, mtval)
  mcycle    := Mux(mcycle_en    , wdata, mcycle)
  minstret  := Mux(minstret_en  , wdata, minstret)
  // todo map pmpcfg[0~15]
  val illegalMret = ena && is_mret && (currentPriv < mode_m)
  val illegalSret = ena && is_sret && (currentPriv < mode_s)
  val illegalSModeSret = ena && is_sret && (currentPriv === mode_s) && mstatus.TSR.asBool // TSR is 1 raise an illegal instr exception, see RISCV-privile 3.1.6.5
  val tvmNotPermit = (currentPriv === mode_s) && mstatus.TVM.asBool
  val accessPermitted = !(csrAddr === CsrAddr.satp && tvmNotPermit)


  val isIllegalAddr = rdata === 0.U // TODO the list is not complele, if check all addr should make a new map
  val isIllegalAccess
  val isIllegalPrivOp = illegalMret || illegalSret || illegalSModeSret // TODO assigned to nowhere in XS?

  when(ena && is_mret && !illegalMret){
    currentPriv  := mstatus.MPP
    status.PIE.M := true.B
    status.IE.M  := mstatus.PIE.M
    status.MPP   := (if (supportUser) mode_u else mode_m)
  }
  when(ena && is_sret && !illegalSret && !illegalSModeSret){
    currentPriv  := Cat(0.U,sstatus.SPP)       // SPP is 1bit width
    status.PIE.S := true.B
    status.IE.S  := sstatus.PIE.S
    status.SPP   := (if (supportUser) mode_u else mode_s)
  }

  val csrExceptionVec = WireInit(ExceptionVec.apply())
  csrExceptionVec.map(_ := false.B) // close
  csrExceptionVec(Breakpoint) := ena && isE
  csrExceptionVec(UECall) := ena && (currentPriv === mode_m) && is_ecall
  csrExceptionVec(SECall) := ena && (currentPriv === mode_u) && is_ecall
  csrExceptionVec(MECall) := ena && (currentPriv === mode_s) && is_ecall


  // 中断相关定义

  def legalizePrivilege(priv: UInt): UInt =
    if (supportUser)
      Fill(2, priv(0))
    else
      Privilege.Level.M

  def real_epc () : UInt = {
    MuxLookup(currentPriv, 0.U, Array(
      mode_m -> mepc
      // todo: add mode s&u
    ))
  }

  def real_tvec (x_mode: UInt) : UInt = {
    MuxLookup(x_mode, 0.U, Array(
      XtvecMode.Direct -> Cat(mtvec_base(61,0), 0.U(2.W)),
      XtvecMode.Vectored -> Cat(mtvec_base + mcause, 0.U(2.W))
    ))
  }


private val trap = WireInit(0.U.asTypeOf(new TrapIO))
  BoringUtils.addSink(trap, "ROBTrap")

  private val interruptVec = mie(11, 0) & mip.asUInt()(11,0) & Fill(12, trap.mstatus.asTypeOf(new StatusStruct).IE.M)
  private val interruptValid = interruptVec.asUInt.orR()
  BoringUtils.addSource(interruptVec, "interruptVec")

  private val curInterruptPc = real_tvec(mtvec_mode)
  private val curInterruptNo = Mux(trap.interruptValid, PriorityEncoder(trap.interruptVec), 0.U)
  private val curInterruptCause = (trap.interruptValid.asUInt << (XLEN - 1).U).asUInt | curInterruptNo


  // --------------------------- 时钟中断 --------------------------
  val mtip = WireInit(false.B)
  BoringUtils.addSink(mtip, "mtip")
  BoringUtils.addSink(mtime, "mtime")
  ip.t.M := mtip


  // --------------------------- 中断处理 --------------------------
  when (currentPriv === mode_m) {
    // ip.t.M:      mtip: M mode出现时钟中断
    // ie.t.M:      mtie: M mode允许时钟中断
    // status.IE.M: mie:  M mode允许中断
    when(trap.interruptValid) {
      // 暂定实时响应中断
      mepc    := trap.epc
      mcause  := curInterruptCause
      status.IE.M := false.B         // xIE设为0，跳转过程中不响应中断
      status.PIE.M := status.IE.M
      status.MPP  := currentPriv
    }
  }




}

trait CsrRegDefine extends Config {
  class PrivilegeMode extends Bundle {
    val M: Bool = Output(Bool())
    val H: Bool = Output(Bool())
    val S: Bool = Output(Bool())
    val U: Bool = Output(Bool())
  }
  class InterruptField extends Bundle {
    val e = new PrivilegeMode
    val t = new PrivilegeMode
    val s = new PrivilegeMode
  }
  object FloatDirtyStatus {
    val off :: initial :: clean :: dirty :: Nil = Enum(4)
    def isOff(FS: UInt)   : Bool = FS === off
    def isDirty(FS: UInt) : Bool = FS === dirty
  }

  object ExtensionDirtyStatus {
    val all_off :: some_on :: some_clean :: some_dirty :: Nil = Enum(4)
    def isOff(XS: UInt)   : Bool = XS === all_off
    def isDirty(XS: UInt) : Bool = XS === some_dirty
  }
  //  CT910 definiton for MSTATUS register is listed as follows
  //  ===============================================================
  //  |63|62 40| 39|38 36|35 34|33 32|31 25|24 23| 22|21| 20| 19| 18|
  //  +--+-----+---+-----+-----+-----+-----+-----+---+--+---+---+---+
  //  |SD| Res |MPV| Res | SXL | UXL | Res | VS  |TSR|TM|TVM|MXR|SUM|
  //  ===============================================================
  //  ===================================================================
  //  | 17 |16 15|14 13|12 11|10 9| 8 |  7 | 6 |  5 | 4 | 3 | 2 | 1 | 0 |
  //  +----+-----+-----+-----+----+---+----+---+----+---+---+---+---+---+
  //  |MPRV| Res | FS  | MPP | Res|SPP|MPIE|Res|SPIE|Res|MIE|Res|SIE|Res|
  //  ===================================================================
  class StatusStruct extends Bundle with Config{
    val SD    : UInt = Output(UInt(1.W))                                         // float unit, vec unit, extension unit status: 1 - dirty, 0 - not dirty
    val PAD0  : UInt = if (MXLEN == 64) Output(UInt((MXLEN - 37).W)) else null   //
    val SXL   : UInt = if (MXLEN == 64) Output(UInt(2.W)) else null              // u_mode reg width, fix value is 2
    val UXL   : UInt = if (MXLEN == 64) Output(UInt(2.W)) else null              // s_mode reg width, fix value is 2
    val PAD1  : UInt = if (MXLEN == 64) Output(UInt(9.W)) else Output(UInt(8.W)) // TODO : in XT910 [24,23] is VS - vector unit status
    val TSR   : UInt = Output(UInt(1.W))                                         // trap sret. s_mode: 1 - execute inster-SRET, produce illegal instr exception; 0 - exe but not produce
    val TW    : UInt = Output(UInt(1.W))                                         // timeout-waiting, s_mode: 1 - execute LowPowerMode inster-WFI, produce illegal instr exception; 0 - exe but not produce
    val TVM   : UInt = Output(UInt(1.W))                                         // trap virtual mem, in s_mode, 1 - can r&w reg-satp and exe instr-SFENCE, produce illegal instr exception; 0 - can r&w reg-satp and exe instr-Sfence
    val MXR   : UInt = Output(UInt(1.W))                                         // 1 - load can access BOTH exe-able & readable virtual memory; 0 - load can access ONLY readable virtual memory
    val SUM   : UInt = Output(UInt(1.W))                                         // 1 - load, store, fetch can access virtual memory in u_mode; 0 - can't
    val MPRV  : UInt = Output(UInt(1.W))                                         // modify priv: 1 - load&store executed depend on MPP-Priv; 0 - load&store executed depend on Current-Priv
    val XS    : UInt = Output(UInt(2.W))                                         // 0
    val FS    : UInt = Output(UInt(2.W))                                         // FU status: 2'b00 - off; 2'b01 - init; 2'b10 - clean; 2'b11 - dirty,
    val MPP   : UInt = Output(UInt(2.W))                                         // preserve Priv-status before start exception: 2'b00 - u_mode; 2'b01 - s_mode; 2'b11 - m_mode; can be reset to 2'b11
    val HPP   : UInt = Output(UInt(2.W))                                         // unused
    val SPP   : UInt = Output(UInt(1.W))                                         // XT910 is 2bits, but PP only can be lower just 2 modes: 1'b0 - u_mode; 1'b1 - s_mode; can be reset to 1'b1
    val PIE   = new PrivilegeMode                                                // preserve XIE value before start interrupt, can be reset
    val IE    = new PrivilegeMode                                                // x_mode interrupt enable, can be reset, will be set as PIE when exit interrupt
  }
  protected val CSR_DATA_W: Width = MXLEN.W

  class DcsrStruct extends Bundle {
    val xdebugver = Output(UInt(2.W))
    val zero4 = Output(UInt(2.W))
    val zero3 = Output(UInt(12.W))
    val ebreakm = Output(Bool())
    val ebreakh = Output(Bool())
    val ebreaks = Output(Bool())
    val ebreaku = Output(Bool())
    val stepie = Output(Bool()) // 0
    val stopcycle = Output(Bool())
    val stoptime = Output(Bool())
    val cause = Output(UInt(3.W))
    val v = Output(Bool()) // 0
    val mprven = Output(Bool())
    val nmip = Output(Bool())
    val step = Output(Bool())
    val prv = Output(UInt(2.W))
  }

  // Supervisor Trap Handling
  // 0x100, 0x104~0x106
  val ie             : InterruptField = RegInit(0.U.asTypeOf(new InterruptField))   // interrupt enable
  val sie            : UInt = WireInit(ie.asUInt())                                 // cat {ie_e_x,ie_t_x,ie_s_x}
  val stvec          : UInt = RegInit(0.U(CSR_DATA_W))                              // set trap vector, such as pc position, 0-direct to BASE, 1-vec to BASE+4*cause, >=2-reserved
  val scounteren     : UInt = RegInit(0.U(CSR_DATA_W))

  // Supervisor Configuration
  val senvcfg   : UInt  = RegInit(0.U(CSR_DATA_W))

  // Supervisor Trap Handling
  // 0x140~0x144
  val sscratch       : UInt = RegInit(0.U(CSR_DATA_W))                               // temp store a Word data
  val sepc           : UInt = RegInit(0.U(CSR_DATA_W))                               // exception pc
  val scause         : UInt = RegInit(0.U(CSR_DATA_W))                               // exception cause
  val stval          : UInt = RegInit(0.U(CSR_DATA_W))                               // additional trap message, such as excp addr, excp pc
  val ip                    = WireInit(0.U.asTypeOf(new InterruptField))             // interrupt pending, list the processing interrupt
  val sip            : UInt = WireInit(ip.asUInt())

  // Supervisor Protection and Translation
  val satp         : UInt  = RegInit(0.U(CSR_DATA_W))

  // Debug/Trace Registers
  val scontext     : UInt  = RegInit(0.U(CSR_DATA_W))


  // Machine Information Registers
  // 0xF11~0xF14
  val mvendorid    : UInt = RegInit(UInt(CSR_DATA_W), 0.U)                          // read-only register providing the JEDEC manufacturer ID
  val marchid      : UInt = RegInit(UInt(CSR_DATA_W), VendorID.U)                   // provides a unique encoding of the version of the processor implementation
  val mimpid       : UInt = RegInit(UInt(CSR_DATA_W), ArchitectureID.U)             // the hardware thread running the code
  val mhartid      : UInt = RegInit(UInt(CSR_DATA_W), ImplementationID.U)           // the read-only pointer pointing to the platform config structure, 0 for not supported

  // 0x300~0x306
  val status       : StatusStruct = RegInit(0.U.asTypeOf(new StatusStruct))
  // val misaInitVal = MXL << (MXLEN - 2) | getMisaExt(ISAEXT)
  //  |XLEN - 1 XLEN - 2|XLEN - 3 26|25  0|
  //  +-----------------+-----------+-----+
  //          BASE      |   zero    | EXT |
  val misa           : UInt = RegInit(UInt(CSR_DATA_W), MISA.U)
  val medeleg        : UInt = RegInit(0.U(CSR_DATA_W))
  val mideleg        : UInt = RegInit(0.U(CSR_DATA_W))                              // judge which interrupt should be delegate to S-mode
//  val ie             : InterruptField = RegInit(0.U.asTypeOf(new InterruptField))   // interrupt enable
  val mie            : UInt = WireInit(ie.asUInt())                                 // cat {ie_e_x,ie_t_x,ie_s_x}
  val mtvec          : UInt = RegInit(0.U(CSR_DATA_W))                              // set trap vector, such as pc position, 0-direct to BASE, 1-vec to BASE+4*cause, >=2-reserved
  val mcounteren     : UInt = RegInit(0.U(CSR_DATA_W))

  // Machine Trap Handling
  // 0x340~0x344
  val mscratch       : UInt = RegInit(0.U(CSR_DATA_W))                               // temp store a Word data
  val mepc           : UInt = RegInit(0.U(CSR_DATA_W))                               // exception pc
  val mcause         : UInt = RegInit(0.U(CSR_DATA_W))                               // exception cause
  val mtval          : UInt = RegInit(0.U(CSR_DATA_W))                               // additional trap message, such as excp addr, excp pc
//  val ip                    = WireInit(0.U.asTypeOf(new InterruptField))             // interrupt pending, list the processing interrupt
  val mip            : UInt = WireInit(ip.asUInt())

  // Machine Memory Protection
  // Configure the appropriate registers to grant or deny read, write, and execute permissions.
  // 0x3A0~0x3A3, 0x3B0~0x3BF
  val pmpcfg         : Vec[UInt] = RegInit(VecInit(Seq.fill(4)(0.U(CSR_DATA_W))))
  val pmpaddr        : Vec[UInt] = RegInit(VecInit(Seq.fill(16)(0.U(CSR_DATA_W))))

  // Machine Counters and Timers
  // 0xB00~0xB1F
  val mhpmcounter    : Vec[UInt] = RegInit(VecInit(Seq.fill(32)(0.U(CSR_DATA_W))))
  val mcycle         : UInt      = mhpmcounter(0)
  val mtime          : UInt      = WireInit(0.U)
  val minstret       : UInt      = mhpmcounter(2)

  // Machine Counter Setup
  // 0x320, 0x323~0x33F
  val mhpmevent      : Vec[UInt] = RegInit(VecInit(Seq.fill(32)(0.U(CSR_DATA_W))))
  val mcountinhibit  : UInt      = mhpmevent(0)

  // Debug/Trace Registers(shared with Debug Mode)
  // 0x7A0~0x7A3
  val tselect       : UInt = RegInit(0.U(CSR_DATA_W))
  val tdata1        : UInt = RegInit(0.U(CSR_DATA_W))
  val tdata2        : UInt = RegInit(0.U(CSR_DATA_W))
  val tdata3        : UInt = RegInit(0.U(CSR_DATA_W))

  // Debug Mode Registers
  // 0x7B0~0x7B3
  val dcsr          : UInt = RegInit(0.U(CSR_DATA_W))
  val dpc           : UInt = RegInit(0.U(CSR_DATA_W))
  val dscratch0     : UInt = RegInit(0.U(CSR_DATA_W))
  val dscratch1     : UInt = RegInit(0.U(CSR_DATA_W))

  /** sub field in CSRs */

  // mtvec
  val mtvec_base    : UInt = mtvec(MXLEN-1, 2)
  val mtvec_mode    : UInt = mtvec(1, 0)
  object XtvecMode {
    def Direct : UInt = 0.U(2.W)
    def Vectored : UInt = 1.U(2.W)
  }


  val mstatus   = WireInit(0.U.asTypeOf(new StatusStruct))
  mstatus.UXL   := (if(supportUser)  (log2Ceil(UXLEN)-4).U else 0.U)
  mstatus.SXL   := (if(supportSupervisor) (log2Ceil(SXLEN)-4).U else 0.U)
  mstatus.SPP   := (if(!supportSupervisor) 0.U else status.SPP)
  mstatus.MPP   := (if(!supportUser) Privilege.Level.M else status.MPP)
  mstatus.IE.U  := (if(!supportUser) 0.U else status.IE.U)
  mstatus.IE.S  := (if(!supportSupervisor) 0.U else status.IE.S)
  mstatus.IE.M  := status.IE.M
  mstatus.PIE.U := (if(!supportUser) 0.U else status.PIE.U)
  mstatus.PIE.S := (if(!supportSupervisor) 0.U else status.PIE.S)
  mstatus.PIE.M := status.PIE.M
  mstatus.FS    := status.FS
  mstatus.SD    := FloatDirtyStatus.isDirty(status.FS) || ExtensionDirtyStatus.isDirty(status.XS)

  val sstatus   = WireInit(0.U.asTypeOf(new StatusStruct))
  sstatus.FS    := status.FS
  sstatus.SD    := FloatDirtyStatus.isDirty(status.FS) || ExtensionDirtyStatus.isDirty(status.XS)

  val readOnlyMap = List (
    CsrAddr.mvendorid   ->  mvendorid   ,
    CsrAddr.marchid     ->  marchid     ,
    CsrAddr.mimpid      ->  mimpid      ,
    CsrAddr.mhartid     ->  mhartid
    //  不可读mip，避免difftest错误，mip仅仅CPU内部使用，对软件不可见
    //    CsrAddr.mip         ->  mip         ,
  )
  val readWriteMap = List (
    CsrAddr.mstatus     ->  mstatus.asUInt(),
    CsrAddr.misa        ->  misa        ,
    CsrAddr.medeleg     ->  medeleg     , // 异常委托寄存器，将m处理的异常委托给更低的特权级
    CsrAddr.mideleg     ->  mideleg     , // 中断委托寄存器，将m处理的中断委托给更低的特权级
    CsrAddr.mie         ->  mie         ,
    CsrAddr.mtvec       ->  mtvec       ,
    CsrAddr.mcounteren  ->  mcounteren  ,
    CsrAddr.mscratch    ->  mscratch    ,
    CsrAddr.mepc        ->  mepc        ,
    CsrAddr.mcause      ->  mcause      ,
    CsrAddr.mtval       ->  mtval       ,
    // todo map pmpcfg[0~15]
    CsrAddr.mcycle      ->  mcycle      ,
    CsrAddr.minstret    ->  minstret
    // todo map mhpmcounter[3~31]
    // todo map Machine Counter Setup, Debug/Trace Registers, Debug Mode Registers
  )
}

  private object CsrAddr {
    def ADDR_W    : Width = 12.W
    // CSR regs encoding: |11         10|9              8|7       0|
    // -------------------+-------------+----------------+---------+-
    //                    |  R&W access |Priv-mode access|

    // Supervisor Trap Handling
    // 0x100, 0x104~0x106
    def sstatus  : UInt  = 0x100.U(ADDR_W)
    def sie       : UInt  = 0x104.U(ADDR_W)
    def stvec     : UInt  = 0x105.U(ADDR_W)
    def scounteren: UInt  = 0x106.U(ADDR_W)

    // Supervisor Configuration
    def senvcfg   : UInt  = 0x10A.U(ADDR_W)

    // Supervisor Trap Handling
    // 0x140~0x144
    def sscratch  : UInt  = 0x140.U(ADDR_W)
    def sepc      : UInt  = 0x141.U(ADDR_W)
    def scause    : UInt  = 0x142.U(ADDR_W)
    def stval     : UInt  = 0x143.U(ADDR_W)
    def sip       : UInt  = 0x144.U(ADDR_W)

    // Supervisor Protection and Translation
    def satp      : UInt  = 0x180.U(ADDR_W)

    // Debug/Trace Registers
    def scontext  : UInt  = 0x5A8.U(ADDR_W)

    // Machine Information Registers
    // 0xF11~0xF14
    def mvendorid : UInt  = 0xF11.U(ADDR_W)
    def marchid   : UInt  = 0xF12.U(ADDR_W)
    def mimpid    : UInt  = 0xF13.U(ADDR_W)
    def mhartid   : UInt  = 0xF14.U(ADDR_W)

    // Machine Trap Setup
    // 0x300~0x306
    def mstatus   : UInt  = 0x300.U(ADDR_W)
    def misa      : UInt  = 0x301.U(ADDR_W)
    def medeleg   : UInt  = 0x302.U(ADDR_W)
    def mideleg   : UInt  = 0x303.U(ADDR_W)
    def mie       : UInt  = 0x304.U(ADDR_W)
    def mtvec     : UInt  = 0x305.U(ADDR_W)
    def mcounteren: UInt  = 0x306.U(ADDR_W)

    // Machine Trap Handling
    // 0x340~0x344
    def mscratch  : UInt  = 0x340.U(ADDR_W)
    def mepc      : UInt  = 0x341.U(ADDR_W)
    def mcause    : UInt  = 0x342.U(ADDR_W)
    def mtval     : UInt  = 0x343.U(ADDR_W)
    def mip       : UInt  = 0x344.U(ADDR_W)
    // def mtinst    : UInt  = 0x34A.U(ADDR_W)
    // def mtval2     : UInt  = 0x34B.U(ADDR_W)

    // Machine Memory Protection
    // 0x3A0~0x3A3, 0x3B0~0x3BF
    def pmpcfg(idx: Int) : UInt = {
      require(idx >= 0 && idx < 4)
      (0x3A0 + idx).U(ADDR_W)
    }
    def pmpaddr(idx: Int) : UInt = {
      require(idx >= 0 && idx < 16)
      (0x3B0 + idx).U(ADDR_W)
    }

    // Machine Counters and Timers
    // 0xB00~0xB1F
    def mcycle    : UInt  = 0xB00.U(ADDR_W)
    def mtime     : UInt  = 0xB01.U(ADDR_W)
    def minstret  : UInt  = 0xB02.U(ADDR_W)
    def mhpmcounter(idx : Int) : UInt = {
      require(idx >= 3 && idx < 32)
      (0xB00 + idx).U(ADDR_W)
    }

    // Machine Counter Setup
    // 0x320, 0x323~0x33F
    def mcountinhibit   : UInt = 0x320.U(ADDR_W)
    def mhpmevent(idx: Int) : UInt = {
      require(idx >= 3 && idx < 32)
      (0x320 + idx).U(ADDR_W)
    }

    // Debug/Trace Registers(shared with Debug Mode)
    // 0x7A0~0x7A3
    val tselect   : UInt  = 0x7A0.U(ADDR_W)
    val tdata1    : UInt  = 0x7A1.U(ADDR_W)
    val tdata2    : UInt  = 0x7A2.U(ADDR_W)
    val tdata3    : UInt  = 0x7A3.U(ADDR_W)

    // Debug Mode Registers
    // 0x7B0~0x7B3
    val dcsr      : UInt  = 0x7B0.U(ADDR_W)
    val dpc       : UInt  = 0x7B1.U(ADDR_W)
    val dscratch0 : UInt  = 0x7B2.U(ADDR_W)
    val dscratch1 : UInt  = 0x7B3.U(ADDR_W)
  }

  object Privilege extends Config {
    object Level {
      def U : UInt = "b00".U(2.W)
      def S : UInt = "b01".U(2.W)
      def H : UInt = "b10".U(2.W)
      def M : UInt = "b11".U(2.W)
    }
    object Access {
      def RW : UInt = "b00".U(2.W)
      def RO : UInt = "b11".U(2.W)
    }
    object FieldSpec {
      /** Reserved Writed Preserve Values, Reads Ingore Values */
      def WPRI : UInt = "b00".U(2.W)
      /** Write/Read Only Legal Values */
      def WLRL : UInt = "b01".U(2.W)
      /** Write Any Values, Reads Legal Values */
      def WARL : UInt = "b11".U(2.W)
    }
    def supportUser       : Boolean = ISAEXT.support('U')
    def supportSupervisor : Boolean = ISAEXT.support('S')
  }

