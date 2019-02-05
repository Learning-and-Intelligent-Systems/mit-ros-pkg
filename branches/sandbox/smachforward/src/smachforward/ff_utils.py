#!/usr/bin/env python
import os

def generate_domain_str(domain_name, fstate_lst):
    '''
    creates the string domain containing all of the actions
    represented by states in fstate_lst
    and generates a list of all the predicates mentioned in the fstate_lst
    '''
    action_str_lst = list()
    for fstate in fstate_lst:
        name = fstate.get_name_str()
        params = fstate.get_params_str()
        precondition = fstate.get_precondition_str()
        effect = fstate.get_effect_str()
        action_str_lst.append(get_action_def_str(name, params, precondition, effect))
    predicates_str, predicates_lst = get_predicates_str(action_str_lst)
    ret = '(define (domain ' + domain_name + ')\n'
    ret = ret + predicates_str + '\n'
    for action_str in action_str_lst:
        ret = ret + action_str + '\n'
    ret = ret + ')'
    return ret, predicates_lst

def parse_ff_output(output_file_contents):
    '''
    expect contents of the output file as list of string lines
    '''
    found = False
    raw_output_lst = list()
    for line in output_file_contents:
        if line.find('found legal plan') >= 0:
            found = True
            break
    if not found:
        print 'warning: no legal plan found'
        return None
    contents_iter = iter(output_file_contents)
    print 'found legal plan'
    try:
        while True:
            line = contents_iter.next()
            if line.find('step') >= 0:
                raw_output_lst.append(line)
                while True:
                    line = contents_iter.next()
                    if line.find(':') >= 0:
                        raw_output_lst.append(line)
                    else:
                        raise StopIteration
    except StopIteration:
        pass
    output_lst = list()
    for line in raw_output_lst:
        print line
        tokens = line.split(':')
        token = tokens[1]
        output_lst.append(token.strip())
    return output_lst

def execute_ff(domain_str, problem_str, prepend=None, 
               domain_file_name='ff_domain.pddl', problem_file_name='ff_problem.pddl', 
               swp_file_name='ff_swp_file.txt'):
    '''
    writes the domain and problem strings to files and runs them through ff.
    a temporary swap file is created to store the ff output.
    ff output is parsed into an ordered list of string actions.
    if no plan can be made, None is returned.
    '''
    domain_file_name = domain_file_name if not prepend else str(prepend) + str(domain_file_name)
    problem_file_name = problem_file_name if not prepend else str(prepend) + str(problem_file_name)
    swp_file_name = swp_file_name if not prepend else str(prepend) + str(swp_file_name)
    domain_file = open(domain_file_name, 'w')
    domain_file.write(domain_str)
    domain_file.close()
    problem_file = open(problem_file_name, 'w')
    problem_file.write(problem_str)
    problem_file.close()
    ff_call = 'ff -o ' + str(domain_file_name) + ' -f ' + str(problem_file_name) + ' > ' + str(swp_file_name)
    os.system(ff_call)
    output_file = open(swp_file_name, 'r')
    output_file_contents = output_file.readlines()
    return parse_ff_output(output_file_contents)

def get_action_def_str(name_str, params_str, precondition_str, effect_str):
    '''
    returns the action string generated from inputs.  example:
    (:action grab :parameters (?x ?y) :precondition (and (cat ?x) (dog ?y)) :effect (not ?y))
    '''
    ret = '(:action ' + name_str + '\n'
    ret = ret + '\t:parameters ' + params_str + '\n'
    ret = ret + '\t:precondition ' + precondition_str + '\n'
    ret = ret + '\t:effect ' + effect_str + ')\n'
    return ret

def get_smallest_nests(string, nests=None):
    ''' 
    returns a list of strings representing the smallest nests
    where the first word in each string is the operator
    and the rest of words are the arguments
    parens, and, or, not are filtered out
    '''
    lparen = None
    if not nests:
        nests = list()
    for i, char in enumerate(string):
        if char is '(' and lparen is None:
            lparen = i
        elif char is '(' and lparen is not None:
            return get_smallest_nests(string[i::], nests)
        elif char is ')' and lparen is not None:
            nest = string[lparen:i].strip('()')
            if nest[0] is not '?':
                nests.append(nest)
            return get_smallest_nests(string[i::], nests)
    return nests

def get_predicates_str(action_str_lst):
    '''
    crawls through the list of action strings and accumulates all predicates used
    in the preconditions and effects.  returns a string of predicates:
    (:predicates (ingred ?x) (dog ?y))
    '''
    operators = dict()
    for action_str in action_str_lst:
        nests = get_smallest_nests(action_str)
        for nest in nests:
            token = nest.split()
            if (token[0] is 'and') or (token[0] is 'or') or (token[0] is 'not'):
                continue
            else:
                num_args = len(token) - 1
                if token[0] in operators:
                    if operators[token[0]] is not num_args:
                        print 'warning: argument number mismatch'
                        print 'in dict for', token[0],' ', operators[token[0]]
                        print 'num_args', num_args
                else:
                    operators[token[0]] = num_args
    okeys = operators.keys()
    okeys.sort()
    alphabet = 'abcdefghijklmnopqrstuvwxyz'
    predicate_lst = list()
    for operator in okeys:
        argument_string = ''
        for i in range(0, operators[operator]):
            argument_string = argument_string + ' ?' + alphabet[i] + ' '
        predicate = '(' + operator + ' ' + argument_string.strip() + ')'
        predicate_lst.append(predicate)
    ret = '(:predicates '
    for predicate in predicate_lst:
        ret = ret + ' ' + predicate 
    ret = ret + ')'
    return ret, predicate_lst

def get_state_from_domain(state_str, domain_fstates_lst):
    state_str = state_str.lower()
    state_str = state_str.strip()
    for fstate in domain_fstates_lst:
        name = fstate.get_name_str()
        name = name.lower()
        name = name.strip()
        param = fstate.is_parametrized()
        #print 'fstate name: ', name, 'state_str:', state_str, param
        if name == state_str and not fstate.is_parametrized():
            return fstate
    return None

def generate_parametrized_fstate_lst(domain_fstates_lst, ff_plan_lst):
    if not ff_plan_lst:
        print 'error: cannot parametrized based on null list (ff planning failed)'
        return None
    state_params_lst = list()
    for plan_line in ff_plan_lst:
        tokens = plan_line.split()
        if len(tokens) > 1:
            state_params_lst.append((tokens[0], tokens[1::]))
        else:
            state_params_lst.append((tokens[0], list()))
    fstate_lst = list()
    for state_params in state_params_lst:
        print state_params
        state_str = state_params[0]
        fstate = get_state_from_domain(state_str, domain_fstates_lst)
        if not fstate:
            print 'error: could not find state matching this line: ', state_params
            return None
        param_result = fstate.parametrize(state_params[1])
        if not param_result:
            print 'error: parametrization of ', fstate, ' failed'
            return None
        fstate_lst.append(fstate)
    return fstate_lst

def chunk_to_closing_paren(string, return_remainder=False):
    '''
    given a string '(...foo...' will search for a closing
    paren to the initial open paren and will return
    that string
    '''
    if string[0] != '(':
        print 'chunking problem, sentence should start with a paren'
        return ''
    open_count = 0
    index = 0
    for i, char in enumerate(string):
        if char == '(':
            open_count += 1
        elif char == ')':
            if open_count == 1:
                index = i
                break
            else:
                open_count -= 1
    if return_remainder:
        return string[0:index+1], string[index+1::]
    return string[0:index+1]

def parse_function_call(string):
    '''
    given a string function call '(foo a b (bar c))'
    returns the function 'foo' and the arguments as a list
    of strings ['a', 'b', '(bar c)']
    '''
    chunk = chunk_to_closing_paren(string)
    if chunk != string:
        print 'parenthesis closing problem, sentence should start with a paren'
        return None, None
    string = string[0:len(string)-1]
    tokens = string.split(' ')
    function = tokens[0].strip('(')
    arguments = _parsing_helper(tokens[1::])
    #print 'function:', function
    #print 'arguments:', arguments
    return function, arguments

def _parsing_helper(tokens):
    arguments = list()
    for i, token in enumerate(tokens[0::]):
        if token[0] == '(':
            arg, remainder = chunk_to_closing_paren(reduce(lambda x,y: x+' '+y, tokens[i::]), True)
            arguments.append(arg)
            if len(remainder.strip(' ()')) > 0:
                arguments.extend(_parsing_helper(remainder.strip().split(' ')))
            break
        else:
            arguments.append(token)
    return arguments

#def _is_function(token):
#    return len(token) > 0 and token[0] == '('
#
#def recursive_evaulate(function, arguments, fxn_to_eval_predicate):
#    if function == 'and':
#        result = True
#        for arg in arguments:
#            if _is_function(arg):
#                new_function, new_arguments = parse_function_call(arg)
#                result = result and recursive_evaluate(new_function, new_arguments, predicate_estimator)
#                if not result:
#                    break
#        return result
#    elif function == 'or':
#        result = False
#        for arg in arguments:
#            if _is_function(arg):
#                new_function, new_arguments = parse_function_call(arg)
#                result = result or recursive_evaluate(new_function, new_arguments, predicate_estimator)
#                if result:
#                    break
#        return result
#    elif function == 'not':
#        if len(arguments) > 1:
#            print 'too many arguments for not:', arguments
#            return False
#        elif _is_function(arguments[0]):
#            new_function, new_arguments = parse_function_call(arg)
#            return not recursive_evaluate(new_function, new_arguments, predicate_estimator)
#        else:
#            print 'non function passed to not:', arguments
#            return False
#    else:
#        predicate = '('+function + ' '.join(arguments)+')'
#        return fxn_to_eval_predicate(predicate)


















