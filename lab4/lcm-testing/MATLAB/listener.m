%% A program for testing receiving data using lcm
% http://lcm-proj.github.io/tut_matlab.html

javaaddpath ./lcm/lcm.jar
javaaddpath ./lcm/my_types.jar

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();

lc.subscribe('EXAMPLE_int', aggregator);

while true
    disp waiting
    millis_to_wait = 1000;
    msg = aggregator.getNextMessage(millis_to_wait);
    if length(msg) > 0
        disp(sprintf('channel of received message: %s', char(msg.channel)))
        disp(sprintf('raw bytes of received message:'))
        disp(sprintf('%d ', msg.data'))

        m = exlcm.extmsg_t(msg.data);

        disp(sprintf('decoded message:\n'))
        disp([ 'id:   ' sprintf('%d ', m.id) ])
    end
end


