#ifndef TMC5160_H
#define TMC5160_H

typedef struct {

} TMC5160;

void tmc5160_init(void);
void tmc5160_tick(void);

extern TMC5160 tmc5160;

#endif