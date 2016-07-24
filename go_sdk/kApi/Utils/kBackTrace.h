/** 
 * @file    kBackTrace.h
 * @brief   Declares the kBackTrace class. 
 *
 * @internal
 * Copyright (C) 2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_BACK_TRACE_H
#define K_API_BACK_TRACE_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @class   kBackTrace
 * @extends kObject
 * @ingroup kApi-Utils
 * @brief   Represents a snapshot of the active functions on a call stack. 
 */
//typedef kObject kBackTrace;            --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kBackTrace object.
 *
 * The constructed back trace object will be initially empty; use the Capture method to acquire
 * trace information.
 * 
 * @public              @memberof kBackTrace
 * @param   trace       Receives constructed back trace object. 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kBackTrace_Construct(kBackTrace* trace, kAlloc allocator); 

/** 
 * Captures back trace information.
 *
 * If supported by the underlying platform, the set of active functions on the call stack 
 * will be captured. For platforms that do not support back trace, the capture operation will 
 * succeed but the resulting trace will be empty. 
 * 
 * @public              @memberof kBackTrace
 * @param   trace       Back trace object. 
 * @param   skip        Count of recent functions to omit from trace. 
 * @return              Operation status. 
 */
kFx(kStatus) kBackTrace_Capture(kBackTrace trace, kSize skip);

/** 
 * Count of function calls in the captured trace. 
 *
 * @public              @memberof kBackTrace
 * @param   trace       Back trace object. 
 * @return              Count of function calls in trace.
 */
kFx(kSize) kBackTrace_Depth(kBackTrace trace); 

/** 
 * Creates a list of descriptive strings, one for each line in the trace.
 *
 * @public              @memberof kBackTrace
 * @param   trace       Back trace object. 
 * @param   lines       Receives list of descriptive strings.
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kBackTrace_Describe(kBackTrace trace, kArrayList* lines, kAlloc allocator); 

kEndHeader()

#include <kApi/Utils/kBackTrace.x.h>

#endif
