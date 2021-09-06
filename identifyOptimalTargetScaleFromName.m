function scale = identifyOptimalTargetScaleFromName(name)
    if contains(name, 'largetarget', 'IgnoreCase',true)
        scale = 1/1.47;
    elseif contains(name, 'smalltarget', 'IgnoreCase',true)
        scale = 1/1.47 * 0.43;
    elseif contains(name, 'smallesttarget', 'IgnoreCase',true)
        scale = 1/1.47 * 0.2;
    else
        error("Cannot identify the size of the tag from the file name: %s", name)
        scale = 1;
    end
end