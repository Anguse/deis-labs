%% A program for testing receiving data using lcm
% http://lcm-proj.github.io/tut_matlab.html

javaaddpath ./lcm/lcm.jar
javaaddpath ./lcm/my_types.jar

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();

lc.subscribe('EXAMPLE_break', aggregator);

while true
    millis_to_wait = 1000;
    msg = aggregator.getNextMessage(millis_to_wait);
    if length(msg) > 0
        m = exlcm.detectmsg_t(msg.data);
        type = m.type;
        if(type == 'alert')
            display('break!')
        end
    end
end


