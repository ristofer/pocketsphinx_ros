#! /usr/bin/env python


import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import uchile_speech_pocketsphinx.msg

def tester_client(number,mic_name):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('/bender/speech/recognizer/recognizer_action', uchile_speech_pocketsphinx.msg.DoRecognitionAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = uchile_speech_pocketsphinx.msg.DoRecognitionGoal(dictionary="Stage1/gpsr_ii",file='/home/robotica/test_speech/'+mic_name+'/test'+str(number))

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

def format_number(number):
    if number<10:
        return '00'+str(number)
    else:
        return '0'+str(number)

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('tester_client_py')
        test_file = open('/home/robotica/test_speech/test_results.txt','w')
        for i in range(43):
            number = format_number(i)
            print number
            test_file.write(number+'\n')
            result_bender = tester_client(number,'bender_mic')
            print 'Bender Mic'
            test_file.write('Bender Mic\n')
            print result_bender.final_result
            test_file.write(result_bender.final_result+'\n')
            
            result_shot = tester_client(number,'shot')
            print 'Shot Mic'
            test_file.write('Shot Mic'+'\n')
            print result_shot.final_result
            test_file.write(result_shot.final_result+'\n')

        for i in range(60,69):
            number = format_number(i)
            print number
            test_file.write(number+'\n')
            result_bender = tester_client(number,'bender_mic')
            print 'Bender Mic'
            test_file.write('Bender Mic'+'\n')
            print result_bender.final_result
            test_file.write(result_bender.final_result+'\n')
            
            result_shot = tester_client(number,'shot')
            print 'Shot Mic'
            test_file.write('Shot Mic'+'\n')
            print result_shot.final_result
            test_file.write(result_shot.final_result+'\n')  
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
    finally:
        test_file.close()