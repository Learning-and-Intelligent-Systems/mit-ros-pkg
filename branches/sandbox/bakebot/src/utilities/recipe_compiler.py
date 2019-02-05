#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
from dynamicfsm.baking_fsm import *
from dynamicfsm.ingredient_filter import *
import rospy

def generate_fsm(recipe, ingredient_filter, do_detection=True, debug=False, ingreds_in_mb=None, minimize_mixes=False, strict_ingred_types=False):
    assembly = _compile(recipe, filter_ingred_pours=ingreds_in_mb, remove_extra_mixes=minimize_mixes)
    fsm, setup_instructions = _assemble(assembly, ingredient_filter, debug, strict_types=strict_ingred_types)
    print 'fsm generated:', fsm
    print 'setup instructions: \n'
    for inst in setup_instructions:
        print '\t'+inst
    return fsm, setup_instructions
    
def _filter_ingred_pours(recipe_lines, ingreds, skip_unnecessary_mix=True):
    filtered_lines = list()
    for ingred_to_skip in ingreds:
        skip_flag = False
        print '************* skipping ingred: ', ingred_to_skip
        for i, line in enumerate(recipe_lines):
            if ingred_to_skip not in line:
                if skip_flag:
                    if 'mix' in line and skip_unnecessary_mix:
                        print '\n***** skipping mix right after '+str(ingred_to_skip)+' *****\n'
                    else:
                        #print 'adding line: ', line
                        filtered_lines.append(line)
                    skip_flag = False
                else:
                    filtered_lines.append(line)
            else:
                print 'line: ', line
                print '\n***** skipping '+str(ingred_to_skip)+'*****\n'
                skip_flag = True
        recipe_lines = filtered_lines
        #print 'filtered lines: ', filtered_lines
    return filtered_lines

def _remove_extra_mixes(recipe_lines):
    lines_minus_mix = list()
    for line in recipe_lines:
        if 'mix' not in line:
            lines_minus_mix.append(line)
    # now I want to insert a mix before the scrape
    scrapeindex = 0
    for i, line in enumerate(lines_minus_mix):
        if 'scrape' in line:
            print 'scrapeindex', i
            scrapeindex= i
            break
    lines_minus_mix.insert(scrapeindex, 'mix(bowl)')
    return lines_minus_mix


def _compile(recipe, filter_ingred_pours=None, remove_extra_mixes=False):
    recipe_minus_semicolons = recipe.replace(';', '\n')
    lines = recipe_minus_semicolons.splitlines()
    code = list()
    comment_block = False
    for line in lines:
        if line == '/*':
            comment_block = True
        elif line == '*/' and comment_block:
            comment_block = False
        elif not comment_block:
            if len(line) > 0:
                if line[0] is not '#':
                    code.append(line)
    if filter_ingred_pours:
        print 'filtering ingredient pours: ', filter_ingred_pours
        code = _filter_ingred_pours(code, filter_ingred_pours, skip_unnecessary_mix=True)
    if remove_extra_mixes:
        print 'removing extra mixes'
        code = _remove_extra_mixes(code)
    print 'compiled code: '
    for line in code:
        print '\t', line
    return code

def _assemble(assembly, ingredient_filter, debug, strict_types=False):
    ingreds = dict()
    user_code = list()
    bsm = BakingStateMachine(ingredient_filter, debug = debug)
    for line in assembly:
        if 'preheat' in line:
            arg_block = line.split('(')[1]
            args = arg_block.split(',')
            time = args[0].strip(' )')
            user_code.append('Please preheat the oven to ' + time + ' degrees F.')
    for line in assembly:
        # TODO: put error/exception catching
        if ('=' in line) and ('Ingredient' in line):
            ingred_var = line.split('=')[0].strip(' "')
            arg_block = line.split('(')[1]
            args = arg_block.split(',')
            for arg in args:
                arg.strip(' )')
            ingreds[ingred_var] = (args[0].strip(' "'), args[1].strip(' "'))
            user_code.append('Please ensure ' + args[1] + ' ' + args[0] + ' is in the workspace.')
        elif 'pour' in line:
            print 'assembling the pour command for: ', line
            arg_block = line.split('(')[1]
            args = arg_block.split(',')
            ingred_to_pour = args[0].strip(' )')
            try:
                ingred_name, ingred_volume = ingreds[ingred_to_pour]
            except KeyError as e:
                if strict_types:
                    print '\n\n!!!!! ingredient "'+ingred_to_pour+'" is undefined\n\n'
                    raise e
                else:
                    ingred_name = ingred_to_pour
            print 'ingred to pour', ingred_to_pour
            print 'ingred name', ingred_name
            #TODO make sure the ingred filter can support ingred_name
            #TODO should really specify whether to pour in mb or cs
            bsm.deal_ingredient(ingred_name)
        elif 'mix' in line:
            bsm.mix()
        elif 'scrape' in line:
            #TODO may want flags here to only scrape once
            bsm.scrape()
        elif 'bake' in line:
            arg_block = line.split('(')[1]
            args = arg_block.split(',')
            time = args[1].strip(' )')
            bsm.bake(int(time))
        elif 'preheat' in line:
            pass
        else:
            print 'unsupported operation:', line
            bsm.unsupported_operation(line)
    return bsm, user_code
