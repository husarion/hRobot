
#include "Arm.h"
#include "CommandInput.h"

extern Arm hRobot;
extern CommandInput InputData;

char program1[] = "SET P0 J; SET P1 C 255 0 50 0 0; SHOWALL;\n";
char program2[] = "\n";
char program3[] = "\n";
char program4[] = "\n";
char program5[] = "\n";
char program6[] = "\n";
char program7[] = "\n";
char program8[] = "\n"; 

void program1_start(){InputData.AddInstructionStream(program1, UI);}
void program2_start(){InputData.AddInstructionStream(program2, UI);}
void program3_start(){InputData.AddInstructionStream(program3, UI);}
void program4_start(){InputData.AddInstructionStream(program4, UI);}
void program5_start(){InputData.AddInstructionStream(program5, UI);}
void program6_start(){InputData.AddInstructionStream(program6, UI);}
void program7_start(){InputData.AddInstructionStream(program7, UI);}
void program8_start(){InputData.AddInstructionStream(program8, UI);}
