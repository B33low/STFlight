#include "param_base.h"

void param_any_init(ParamAny *p, void *storage, uint16_t size,
                    ParamValidateFn validate, ParamApplyFn apply) {
    state_any_init(&p->base, storage, size);
    p->validate = validate;
    p->apply = apply;
}

bool param_any_set(ParamAny *p, const void *value, uint32_t now_us) {
    if (p->validate && !p->validate(value, p->base.size)) return false;
    state_any_set(&p->base, value, now_us);
    if (p->apply) p->apply(value, p->base.size);
    return true;
}
