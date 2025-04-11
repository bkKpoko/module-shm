#include "mbconfig.h"           /* This goes first in every *.c,*.cc file */

#include "module-shm.h"
#include "filedrv.h"

#include <cstddef>
#include <cstdio>
#include <fcntl.h>
#include <iostream>
#include <cfloat>
#include <ostream>
#include <semaphore.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

#include "dataman.h"
#include "mbdyn.h"
#include "ac/f2c.h"
#include "myassert.h"

//====================================================
//=                                                  =
//=             MODULE SHARED MEMORY INPUT           =
//=                    module_init                   =
//=                                                  =
//====================================================

extern "C" int
module_init(const char *module_name, void *pdm, void *php) {
  SetDriveData("shared" "memory", new ShmDR);
  printf("Shared memory driver initialized\n");
  
  UserDefinedElemRead *rf_shm = new UDERead<ModuleShmOut>;
	if (!SetUDE("shm_out", rf_shm)) {
		delete rf_shm;

		silent_cerr("module-shm-out: "
			"module_init(" << module_name << ") "
			"failed" << std::endl);

		return -1;
	}

  UserDefinedElemRead *rf_verbose = new UDERead<ModuleVerbose>;
	if (!SetUDE("verbose", rf_verbose)) {
		delete rf_verbose;

		silent_cerr("module-verbose: "
			"module_init(" << module_name << ") "
			"failed" << std::endl);

		return -1;
	}
	return 0;
}

//====================================================
//=                                                  =
//=                MODULE SHARED MEMORY              =
//=                   ModuleShmOut                   =
//=                                                  =
//====================================================

ModuleShmOut::ModuleShmOut(
	unsigned uLabel, const DofOwner *pDO,
	DataManager* pDM, MBDynParser& HP)
: UserDefinedElem(uLabel, pDO) {
  // help
	if (HP.IsKeyWord("help")) {
		silent_cout(
      "									\n"
      "Module: 	shared memory output						\n"
      "Outputs data to SharedMemory \n"
      "Author: 	Ilyas Khasenov  <i.hasenov@rtc.ru>		\n"
      "Organization:	rtc			\n"
      "		http://www.rtc.ru/				\n"
      "									\n"
      "	All rights reserved						\n"
			<< std::endl);

		if (!HP.IsArg()) {
			/*
			 * Exit quietly if nothing else is provided
			 */
			throw NoErr(MBDYN_EXCEPT_ARGS);
		}
	}

  // Init private data
  dmp = pDM;
  outShmName = HP.GetFileName(); 
  elemNum = HP.GetInt();
  outBufferSize = sizeof(SharedData) + elemNum * sizeof(doublereal);

  printf("Output shared memory name: '%s'\n", outShmName.c_str());

  silent_cout("Joints will be send to SharedMemory : " << elemNum << "\n");
  for (size_t i = 0; i < elemNum; ++i) {
    Elem* el = dmp->ReadElem(HP, Elem::Type::JOINT);
    elements.push_back(el);
    jointsData.push_back(el->iGetPrivDataIdx(HP.GetStringWithDelims()));
    silent_cout("\t(" << el->GetLabel() << ")\n");
  }

  outShmFd = InitSharedMemory(outShmName.c_str(), outBufferSize); 
  outBuffer = (SharedData*)mmap(NULL, outBufferSize, PROT_READ | PROT_WRITE, MAP_SHARED, outShmFd, 0);
  outBuffer->dataSize = elemNum;
  sem_init(&outBuffer->semProduce, 1, 1);
  sem_init(&outBuffer->semConsume, 1, 0);

  silent_cout("MODULE: " << uLabel << " got required data successfully\n");
}

int ModuleShmOut::InitSharedMemory(const char* outShmName, size_t size){
  int fd = shm_open(outShmName, O_CREAT | O_RDWR, 0666);
  ftruncate(fd, size);
  return fd;
}

void ModuleShmOut::Write() const{
  sem_wait(&outBuffer->semProduce);
  for (size_t i = 0; i < outBuffer->dataSize; ++i) {
    outBuffer->data[i] = elements[i]->dGetPrivData(jointsData[i]); 
    // printf("element[%i] = %g\n", i, elements[i]->dGetPrivData(elements[i]->iGetPrivDataIdx("rz")));
  }
  sem_post(&outBuffer->semConsume);
}

ModuleShmOut::~ModuleShmOut(void) {
	// destroy private data
  close(outShmFd);
  munmap(outBuffer, outBufferSize);
  shm_unlink(outShmName.c_str());
	NO_OP;
}
 
void ModuleShmOut::AfterConvergence(const VectorHandler& X, 
		const VectorHandler& XP){
  //   auto end = std::chrono::system_clock::now();
  // auto ms = std::chrono::time_point_cast<std::chrono::milliseconds>(end);
  // auto epoch = ms.time_since_epoch();
  // long long milliseconds = epoch.count();
  //
  // std::cout << "SHM time = " << milliseconds << std::endl;
  Write();
 // printf("ARTER\n");
}

void
ModuleShmOut::Output(OutputHandler& OH) const {
  // Write();
  // auto end = std::chrono::system_clock::now();
  // auto ms = std::chrono::time_point_cast<std::chrono::milliseconds>(end);
  // auto epoch = ms.time_since_epoch();
  // long long milliseconds = epoch.count();
  //
  // std::cout << "SHM time = " << milliseconds << std::endl;
}

void
ModuleShmOut::WorkSpaceDim(integer* piNumRows, integer* piNumCols) const {
	*piNumRows = 0;
	*piNumCols = 0;
}

VariableSubMatrixHandler& 
ModuleShmOut::AssJac(VariableSubMatrixHandler& WorkMat,
	doublereal dCoef, 
	const VectorHandler& XCurr,
	const VectorHandler& XPrimeCurr) {
	// should do something useful
	WorkMat.SetNullMatrix();

	return WorkMat;
}

SubVectorHandler& 
ModuleShmOut::AssRes(SubVectorHandler& WorkVec,
	doublereal dCoef,
	const VectorHandler& XCurr, 
	const VectorHandler& XPrimeCurr) {
	// should do something useful
	WorkVec.ResizeReset(0);

	return WorkVec;
}

unsigned int
ModuleShmOut::iGetNumPrivData(void) const {
	return 0;
}

int
ModuleShmOut::iGetNumConnectedNodes(void) const {
	return 0;
}

void
ModuleShmOut::GetConnectedNodes(std::vector<const Node *>& connectedNodes) const {
	connectedNodes.resize(0);
}

void
ModuleShmOut::SetValue(DataManager *pDM,
	VectorHandler& X, VectorHandler& XP,
	SimulationEntity::Hints *ph) {
	NO_OP;
}

std::ostream&
ModuleShmOut::Restart(std::ostream& out) const {
	return out << "# ModuleShmOut: not implemented" << std::endl;
}

unsigned int
ModuleShmOut::iGetInitialNumDof(void) const {
	return 0;
}

void 
ModuleShmOut::InitialWorkSpaceDim(
	integer* piNumRows,
	integer* piNumCols) const {
	*piNumRows = 0;
	*piNumCols = 0;
}

VariableSubMatrixHandler&
ModuleShmOut::InitialAssJac(
	VariableSubMatrixHandler& WorkMat, 
	const VectorHandler& XCurr) {
	// should not be called, since initial workspace is empty
	ASSERT(0);

	WorkMat.SetNullMatrix();

	return WorkMat;
}

SubVectorHandler& 
ModuleShmOut::InitialAssRes(
	SubVectorHandler& WorkVec,
	const VectorHandler& XCurr) {
	// should not be called, since initial workspace is empty
	ASSERT(0);

	WorkVec.ResizeReset(0);

	return WorkVec;
}

//====================================================
//=                                                  =
//=                SHARED MEMORY DRIVE               =
//=                     ShmDrive                     =
//=                                                  =
//====================================================

/* ShmDrive - begin */

static const std::vector<doublereal> v0;

ShmDrive::ShmDrive(unsigned int uL,
		const DriveHandler* pDH,
		const char* const sFileName,
		integer ins, integer ind,
		doublereal t0, doublereal dt)
    : FileDrive(uL, pDH, sFileName, ind, v0),
      dT0(t0), dDT(dt), iNumSteps(ins) {

  ASSERT(iNumDrives > 0);
  ASSERT(sFileName != NULL);
  ASSERT(dDT > 0.);

  printf("Input shared memory name: '%s'\n", sFileName);
  size_t shm_size = sizeof(SharedData) + iNumDrives * sizeof(doublereal);
  inShmFd = shm_open(sFileName, O_RDWR, 0666);
  inBuffer = (SharedData*)mmap(NULL, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, inShmFd, 0); 

  if (iNumDrives != (int)inBuffer->dataSize){
    printf("Size of input data not capable with recvested num drives\n");
    printf("iNumDrives = %i", iNumDrives);
    printf("dataSize = %zi", inBuffer->dataSize);
		throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }
  
	// ServePending(pDH->dGetTime());
}

ShmDrive::~ShmDrive(void) {
  munmap(inBuffer, sizeof(SharedData));
  shm_unlink(sFileName.c_str());
}

/* Scrive il contributo del DriveCaller al file di restart */
std::ostream&
ShmDrive::Restart(std::ostream& out) const {
	return out << "0. /* ShmDrive: not implemented yet! */"
		<< std::endl;
}

void
ShmDrive::ServePending(const doublereal& t) {
  // printf("PENIS\n");
  sem_wait(&inBuffer->semConsume);
  for (int i = 0; i < iNumDrives; ++i){
    pdVal[i + 1] = inBuffer->data[i];
  }
  sem_post(&inBuffer->semProduce);
}


//====================================================
//=                                                  =
//=                SHARED DRIVE READ                 =
//=                   ShmDR::Read                    =
//=                                                  =
//====================================================


Drive *
ShmDR::Read(unsigned uLabel, const DataManager *pDM, MBDynParser& HP) {
	integer isteps = -1;
	integer idrives = HP.GetInt();
	if (idrives <= 0) {
		silent_cerr("ShmDrive(" << uLabel << "): "
			"invalid channels number " << idrives
			<< " at line " << HP.GetLineData()
			<< std::endl);
		throw ErrGeneric(MBDYN_EXCEPT_ARGS);
	}

  doublereal t0 = HP.GetReal();
	if (t0 < 0) {
		silent_cerr("ShmDrive(" << uLabel << "): "
			"invalid intial time " << t0
			<< " at line " << HP.GetLineData()
			<< std::endl);
		throw ErrGeneric(MBDYN_EXCEPT_ARGS);
	}

  doublereal dt = HP.GetReal();
	if (dt <= 0) {
		silent_cerr("ShmDrive(" << uLabel << "): "
			"invalid delta time " << dt 
			<< " at line " << HP.GetLineData()
			<< std::endl);
		throw ErrGeneric(MBDYN_EXCEPT_ARGS);
	}

	const char* filename = HP.GetFileName();

	Drive* pDr = NULL;
	SAFENEWWITHCONSTRUCTOR(pDr,
			ShmDrive,
			ShmDrive(uLabel, pDM->pGetDrvHdl(),
				filename, isteps, idrives, t0, dt));

	return pDr;
}



#include <sys/ioctl.h>
//====================================================
//=                                                  =
//=                  MODULE VERBOSE                  =
//=                   ModuleVerbose                  =
//=                                                  =
//====================================================

ModuleVerbose::ModuleVerbose(
	unsigned uLabel, const DofOwner *pDO,
	DataManager* pDM, MBDynParser& HP)
: UserDefinedElem(uLabel, pDO) {
  // help
	if (HP.IsKeyWord("help")) {
		silent_cout(
      "									\n"
      "Module: 	verbose						\n"
      "prints percentage of calculation completed \n"
      "Author: 	Ilyas Khasenov  <i.hasenov@rtc.ru>		\n"
      "Organization:	rtc			\n"
      "		http://www.rtc.ru/				\n"
      "									\n"
      "	All rights reserved						\n"
			<< std::endl);

		if (!HP.IsArg()) {
			/*
			 * Exit quietly if nothing else is provided
			 */
			throw NoErr(MBDYN_EXCEPT_ARGS);
		}
	}

  
  if (HP.IsKeyWord("time" "step")) {
    dt = HP.GetReal();
  }
  if (HP.IsKeyWord("final" "time")) {
    finalTime = HP.GetReal();
  }

  t = 0;

  ASSERT(dt > 0);
  ASSERT(finalTime > 0);

  printf("t = %g\n", t); 
  printf("ft = %g\n", finalTime); 
  printf("dt = %g\n", dt); 

  ioctl(STDOUT_FILENO, TIOCGWINSZ, &size);
  silent_cout("MODULE: " << uLabel << " got required data successfully\n");
}

ModuleVerbose::~ModuleVerbose(void) {
	// destroy private data
	NO_OP;
}
 
void ModuleVerbose::AfterConvergence(const VectorHandler& X, 
		const VectorHandler& XP){
  snprintf(buf, sizeof(buf), "%.2f", t / finalTime * 100);
  std::string progress_bar = "progress: -->" + std::string(buf) + "%<--";
  
  int width = size.ws_col - progress_bar.length();
  std::cout << "\r" << std::string(width, ' ') << progress_bar << std::flush; 

  t += dt;
}

void
ModuleVerbose::Output(OutputHandler& OH) const {
}

void
ModuleVerbose::WorkSpaceDim(integer* piNumRows, integer* piNumCols) const {
	*piNumRows = 0;
	*piNumCols = 0;
}

VariableSubMatrixHandler& 
ModuleVerbose::AssJac(VariableSubMatrixHandler& WorkMat,
	doublereal dCoef, 
	const VectorHandler& XCurr,
	const VectorHandler& XPrimeCurr) {
	// should do something useful
	WorkMat.SetNullMatrix();

	return WorkMat;
}

SubVectorHandler& 
ModuleVerbose::AssRes(SubVectorHandler& WorkVec,
	doublereal dCoef,
	const VectorHandler& XCurr, 
	const VectorHandler& XPrimeCurr) {
	// should do something useful
	WorkVec.ResizeReset(0);

	return WorkVec;
}

unsigned int
ModuleVerbose::iGetNumPrivData(void) const {
	return 0;
}

int
ModuleVerbose::iGetNumConnectedNodes(void) const {
	return 0;
}

void
ModuleVerbose::GetConnectedNodes(std::vector<const Node *>& connectedNodes) const {
	connectedNodes.resize(0);
}

void
ModuleVerbose::SetValue(DataManager *pDM,
	VectorHandler& X, VectorHandler& XP,
	SimulationEntity::Hints *ph) {
	NO_OP;
}

std::ostream&
ModuleVerbose::Restart(std::ostream& out) const {
	return out << "# ModuleVerbose: not implemented" << std::endl;
}

unsigned int
ModuleVerbose::iGetInitialNumDof(void) const {
	return 0;
}

void 
ModuleVerbose::InitialWorkSpaceDim(
	integer* piNumRows,
	integer* piNumCols) const {
	*piNumRows = 0;
	*piNumCols = 0;
}

VariableSubMatrixHandler&
ModuleVerbose::InitialAssJac(
	VariableSubMatrixHandler& WorkMat, 
	const VectorHandler& XCurr) {
	// should not be called, since initial workspace is empty
	ASSERT(0);

	WorkMat.SetNullMatrix();

	return WorkMat;
}

SubVectorHandler& 
ModuleVerbose::InitialAssRes(
	SubVectorHandler& WorkVec,
	const VectorHandler& XCurr) {
	// should not be called, since initial workspace is empty
	ASSERT(0);

	WorkVec.ResizeReset(0);

	return WorkVec;
}


