#!/usr/bin/python

if __name__ == "__main__": 

    import datetime 
    import smtplib 
    server = smtplib.SMTP("smtp.mail.yahoo.com",587) #'smtp.gmail.com', 587)
    server.starttls()
    server.login("s_arora1987@yahoo.in","dndacnbunktgzcys") #"sonu.1987.arora@gmail.com", "s_arora1987")
    msgsubject = "Notification Logan - Simulation Finished \n"
    msg = "Sent at : "
    msg += str(datetime.datetime.now()) + "\n"
    msg += "Simulation Finished"
    message = 'Subject: {}\n\n{}'.format(msgsubject, msg)
    server.sendmail("s_arora1987@yahoo.in", "sarora@udel.edu", message)#"sonu.1987.arora@gmail.com"
    print("sending Notification")
    server.quit()


