#include "mbconfig.h"           /* This goes first in every *.c,*.cc file */

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <cfloat>
#include <semaphore.h>
#include <string>
#include <vector>

#include "dataman.h"
#include "mbpar.h"
#include "userelem.h"

#include "drive.h"
#include "filedrv.h"


//====================================================
//=                                                  =
//=                MODULE SHARED MEMORY              =
//=                SharedData structure              =
//=                                                  =
//====================================================
typedef struct {
  sem_t semProduce;
  sem_t semConsume;
  size_t dataSize;
  doublereal data[];
} SharedData;

//====================================================
//=                                                  =
//=                MODULE SHARED MEMORY              =
//=                   ModuleShmOut                   =
//=                                                  =
//====================================================

class ModuleShmOut
: public UserDefinedElem {
private:
  std::vector<Elem*> elements; 
  std::vector<unsigned int> jointsData;
  DataManager* dmp;
  unsigned int elemNum;
  bool bPrintStepsNum;

  // shared memory data
  std::string outShmName;
  int outShmFd;
  size_t outBufferSize;
  SharedData* outBuffer;

public:
	ModuleShmOut(unsigned uLabel, const DofOwner *pDO,
		DataManager* pDM, MBDynParser& HP);
	virtual ~ModuleShmOut(void);

	virtual void Output(OutputHandler& OH) const;
	virtual void WorkSpaceDim(integer* piNumRows, integer* piNumCols) const;
	VariableSubMatrixHandler& 
	AssJac(VariableSubMatrixHandler& WorkMat,
         doublereal dCoef, 
         const VectorHandler& XCurr,
         const VectorHandler& XPrimeCurr);
	SubVectorHandler& 
	AssRes(SubVectorHandler& WorkVec,
         doublereal dCoef,
         const VectorHandler& XCurr, 
         const VectorHandler& XPrimeCurr);
	unsigned int iGetNumPrivData(void) const;
	int iGetNumConnectedNodes(void) const;
	void GetConnectedNodes(std::vector<const Node *>& connectedNodes) const;
	void SetValue(DataManager *pDM, VectorHandler& X, VectorHandler& XP,
		            SimulationEntity::Hints *ph);
	std::ostream& Restart(std::ostream& out) const;
	virtual unsigned int iGetInitialNumDof(void) const;
	virtual void 
	InitialWorkSpaceDim(integer* piNumRows, integer* piNumCols) const;
  VariableSubMatrixHandler&
	InitialAssJac(VariableSubMatrixHandler& WorkMat, 
                const VectorHandler& XCurr);
  SubVectorHandler& 
	InitialAssRes(SubVectorHandler& WorkVec, const VectorHandler& XCurr);
	void AfterConvergence(const VectorHandler& X, 
		const VectorHandler& XP);
private:
  void Write() const;
  int InitSharedMemory(const char* outShmName, size_t size);
};


//====================================================
//=                                                  =
//=                 SHARED MEMORY DRIVE              =
//=                       ShmDrive                   =
//=                                                  =
//====================================================
class ShmDrive : public FileDrive {
protected:
	doublereal dT0;
	doublereal dDT;
	integer iNumSteps;
	bool bLinear;
	bool bPadZeroes;
	Bailout boWhen;

public:
	ShmDrive(unsigned int uL, const DriveHandler* pDH,
			const char* const inShmName, integer is, integer id,
			doublereal t0, doublereal dt);
	virtual ~ShmDrive(void);

	/* Scrive il contributo del DriveCaller al file di restart */
	virtual std::ostream& Restart(std::ostream& out) const;

	virtual void ServePending(const doublereal& t);

private:
  SharedData* inBuffer;
  int inShmFd;
};

/* ShmDrive - end */


class DataManager;
class MBDynParser;


//====================================================
//=                                                  =
//=                 SHARED DRIVE READ                =
//=                    ShmDR::Read                   =
//=                                                  =
//====================================================

struct ShmDR : public DriveRead {
public:
	virtual Drive *
	Read(unsigned uLabel, const DataManager *pDM, MBDynParser& HP);
};


//====================================================
//=                                                  =
//=                 MODULE VERBOSE                   =
//=                                                  =
//====================================================


#include <sys/ioctl.h>
class ModuleVerbose
: public UserDefinedElem {
private:
  doublereal finalTime;
  doublereal dt;
  doublereal t;

  char buf[32];
  struct winsize size;


public:
	ModuleVerbose(unsigned uLabel, const DofOwner *pDO,
		DataManager* pDM, MBDynParser& HP);
	virtual ~ModuleVerbose(void);

	virtual void Output(OutputHandler& OH) const;
	virtual void WorkSpaceDim(integer* piNumRows, integer* piNumCols) const;
	VariableSubMatrixHandler& 
	AssJac(VariableSubMatrixHandler& WorkMat,
         doublereal dCoef, 
         const VectorHandler& XCurr,
         const VectorHandler& XPrimeCurr);
	SubVectorHandler& 
	AssRes(SubVectorHandler& WorkVec,
         doublereal dCoef,
         const VectorHandler& XCurr, 
         const VectorHandler& XPrimeCurr);
	unsigned int iGetNumPrivData(void) const;
	int iGetNumConnectedNodes(void) const;
	void GetConnectedNodes(std::vector<const Node *>& connectedNodes) const;
	void SetValue(DataManager *pDM, VectorHandler& X, VectorHandler& XP,
		            SimulationEntity::Hints *ph);
	std::ostream& Restart(std::ostream& out) const;
	virtual unsigned int iGetInitialNumDof(void) const;
	virtual void 
	InitialWorkSpaceDim(integer* piNumRows, integer* piNumCols) const;
  VariableSubMatrixHandler&
	InitialAssJac(VariableSubMatrixHandler& WorkMat, 
                const VectorHandler& XCurr);
  SubVectorHandler& 
	InitialAssRes(SubVectorHandler& WorkVec, const VectorHandler& XCurr);
	void AfterConvergence(const VectorHandler& X, 
		const VectorHandler& XP);
private:
};

