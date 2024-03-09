#include "User.h"

void Limit(fp32 Imput, fp32 LimitCount)
{                                                    
    if (Imput > LimitCount || Imput < -LimitCount)
    {                                                
        Imput = Imput;                          
    }                                                   
    else                                            
    {                                               
        Imput = 0;                                
    }                                               
}

