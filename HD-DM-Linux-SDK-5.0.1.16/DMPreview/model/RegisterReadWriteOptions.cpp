#include "RegisterReadWriteOptions.h"

RegisterReadWriteOptions::RegisterReadWriteOptions()
{
    for (int i = 0 ; i < REGISTER_REQUEST_MAX_COUNT ; ++i){
        m_requestValue[i] = EOF;
        m_requestAddess[i] = EOF;
    }
}
