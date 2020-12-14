    global patrollersuccessrate
    args = [ 'boydsimple_t', ]

    p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)                

    outtraj = ""
    
    outtraj += mapToUse + "\n"
    outtraj += str(patrollersuccessrate)

    (transitionfunc, stderr) = p.communicate(outtraj)

    args = ["boydile", ]
    p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)                
    outtraj = ""
    outtraj += mapToUse + "\n"
    outtraj += transitionfunc + "\n"
    outtraj += lineFoundWeights + "\n"

    f = open(get_home() + "/patrolstudy/toupload/data_ile.log", "a")
    f.write(outtraj)
    f.close()
                
    #(stdout, stderr) = p.communicate(outtraj)
    # Parsing
    
