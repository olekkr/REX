

# Skulle nok være en class men idk hvordan man præcis laver dem hihi
def setState(state, landmark_seen, do_resample):
    # Første state lul
    if state == 'initailize':
        if landmark_seen[2] and landmark_seen[3]:
            state = 'forward'
            do_resample = True
        else:
            state = 'initailize'
    
    elif state == 'searching':
        if landmark_seen[2] and landmark_seen[3]:
            state = 'forward'
        else:
            state = 'searching'

    
    elif state == 'forward':
        state = 'selflocalize'
    
    elif state == 'selflocalize':
        if landmark_seen[2] and landmark_seen[3]:
            state = 'forward'
        else:
            state = 'searching'

    
    return state, landmark_seen, do_resample