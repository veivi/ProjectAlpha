#ifndef INTERRUPT_H
#define INTERRUPT_H

extern uint8_t nestCount;

#define FORBID if(!nestCount++) cli()
#define PERMIT if(!--nestCount) sei()

#endif

