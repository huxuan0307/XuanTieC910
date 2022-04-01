package Core.LSU
import Core.ExceptionConfig.ExceptionVecWidth
import Core.IU.Cp0.Define.Exceptions.ExceptionVec
import Core.IntConfig.{NumLogicRegsBits, NumPhysicRegs, XLEN}
import Core.LsuConfig
import Core.ROBConfig.RobPtrWidth
import chisel3._
import chisel3.util._
//==========================================================
//                        Input
//==========================================================
class Cp0ToStWb extends Bundle with LsuConfig{
  val icgEn = Bool()
  val clkEn  = Bool()
}
class StDaToStWb extends Bundle with LsuConfig{
  val bkptaData = Bool()
  val bkptbData = Bool()
  val instVld   = Bool()
  val iid       = UInt(RobPtrWidth.W)
}
class LdDaToStWb extends Bundle with LsuConfig{
  val addr              = UInt(PA_WIDTH.W)
  val bkptaData         = Bool()
  val bkptbData         = Bool()
  val iid               = UInt(RobPtrWidth.W)
  val instVfls          = Bool()
  val instVld           = Bool()
  val preg              = UInt(NumPhysicRegs.W)
  val pregSignSel       = UInt(PREG_SIGN_SEL.W)
  val vreg              = UInt(NumLogicRegsBits.W)
  val vregSignSel       = UInt(VREG_SIGN_SEL.W)
  val cmpltReq          = Bool()
  val data              = UInt(XLEN.W)
  val dataReq           = Bool()
  val dataReqGateclkEn  = Bool()
  val exptVec           = UInt(ExceptionVecWidth.W)
  val exptVld           = Bool()
  val mtValue           = UInt(PA_WIDTH.W)
  val noSpecHit         = Bool()
  val noSpecMispred     = Bool()
  val noSpecMiss        = Bool()
  val specFail          = Bool()
}
class RbToStWb extends Bundle with LsuConfig{
  val bkptaData   = Bool()
  val bkptbData   = Bool()
  val busErr      = Bool()
  val busErrAddr  = UInt(PA_WIDTH.W)
  val cmpltReq    = Bool()
  val data        = UInt(XLEN.W)
  val dataIid     = UInt(RobPtrWidth.W)
  val dataReq     = Bool()
  val exptGateClk = Bool()
  val exptVld     = Bool()
  val iid         = UInt(RobPtrWidth.W)
  val instVfls    = Bool()
  val preg        = UInt(NumPhysicRegs.W)
  val pregSignSel = UInt(PREG_SIGN_SEL.W)
  val vreg        = UInt(NumLogicRegsBits.W)
  val vregSignSel = UInt(VREG_SIGN_SEL.W)
}
class WmbToStWb extends Bundle with LsuConfig{
  val data         = UInt(XLEN.W)
  val dataAddr     = UInt(PA_WIDTH.W)
  val dataIid      = UInt(RobPtrWidth.W)
  val dataReq      = Bool()
  val instVfls     = Bool()
  val preg         = UInt(NumPhysicRegs.W)
  val pregSignSel  = UInt(PREG_SIGN_SEL.W)
  val vreg         = UInt(NumLogicRegsBits.W)
  val vregSignSel  = UInt(VREG_SIGN_SEL.W)
}
//----------------------------------------------------------
class StoreWbIn extends Bundle with LsuConfig{
  val cp0In  = new Cp0ToStWb
  val stDaIn = new StDaToStWb
  val ldDaIn = new LdDaToStWb
  val rbIn   = new RbToStWb
  val vmbWbDataReg = Bool()
}
//==========================================================
//                        Output
//==========================================================
class StWbToCtrl extends Bundle with LsuConfig {
  val dataVld = Bool()
  val instVld = Bool()
}
class StWbToRb extends Bundle with LsuConfig {
  val cmpltGrnt = Bool()
  val dataGrnt  = Bool()
}
class WbPipe3VregVr extends Bundle with LsuConfig {
  val data  = UInt(XLEN.W)
  val expand= UInt(XLEN.W)
}
class StWbToIdu extends Bundle with LsuConfig {
  val fwdVreg    = ValidIO(UInt(NumLogicRegsBits.W))
  val wbPreg     = ValidIO(UInt(NumPhysicRegs.W))
  val wbPregDup  = Vec(DATA_UPDATE_PATH_WIDTH, ValidIO(UInt(NumPhysicRegs.W)))
  val wbPregExpand = UInt((XLEN+32).W)
  val wbVregVld  = Bool()
  val wbVregDup  = Vec(DATA_UPDATE_PATH_WIDTH, ValidIO(UInt(NumLogicRegsBits.W)))
  val wbFrExpand = UInt((XLEN).W)
  val wbFrData   = ValidIO(UInt((XLEN).W))
  val vregVr     = Vec(2,ValidIO(new WbPipe3VregVr))
}
class StWbToRtu extends Bundle with LsuConfig {

}
//----------------------------------------------------------
class StoreWbOut extends Bundle with LsuConfig{
  val toCtrl = new StWbToCtrl
  val toRb   = new StWbToRb

  val toIdu  = new StWbToIdu


}
//==========================================================
//                          IO
//==========================================================
class StoreWbIO extends Bundle with LsuConfig{
  val in  = Input(new StoreWbIn)
  val out = Output(new StoreWbOut)
}
class StoreWb extends Module with LsuConfig{
  val io = IO(new StoreWbIO)


}
