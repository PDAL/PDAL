/*
 *  Copyright 2011 Steven Watanabe
 *  Distributed under the Boost Software License, Version 1.0.
 *  (See accompanying file LICENSE_1_0.txt or http://www.boost.org/LICENSE_1_0.txt)
 */

#ifndef FUNCTION_SW20111123_H
#define FUNCTION_SW20111123_H

#include "object.h"
#include "frames.h"
#include "lists.h"
#include "parse.h"
#include "strings.h"

typedef struct _function FUNCTION;
typedef struct _stack STACK;

STACK * stack_global( void );
void stack_push( STACK * s, LIST * l );
LIST * stack_pop( STACK * s );

FUNCTION * function_compile( PARSE * parse );
FUNCTION * function_builtin( LIST * ( * func )( FRAME * frame, int flags ), int flags );
void function_refer( FUNCTION * );
void function_free( FUNCTION * );
OBJECT * function_rulename( FUNCTION * );
void function_set_rulename( FUNCTION *, OBJECT * );
void function_location( FUNCTION *, OBJECT * *, int * );
LIST * function_run( FUNCTION * function, FRAME * frame, STACK * s );

FUNCTION * function_compile_actions( const char * actions, OBJECT * file, int line );
void function_run_actions( FUNCTION * function, FRAME * frame, STACK * s, string * out );

void function_done( void );

#endif
