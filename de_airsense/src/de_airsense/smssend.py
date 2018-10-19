#!/usr/bin/env python
# -*- coding: utf8 -*-
""" Автор Титов А.В.titov.aleksandr@edbox.ru 17.02.2015 """
""" Скрипт предназначен для отправки СМС через сайт sms.ru """
""" Принимает следующие аргументы:"""
""" -i или --idsender - id пользователя на sms.ru"""
""" -t или --to - номер телефона получателя в формате 79219996660"""
""" -s или --subject - текст сообщения на латинице"""
from urllib2 import urlopen

def sendsms(idsender,subject,to):
    subject = subject.replace(" ","+")
    url="https://sms.ru/sms/send?api_id=%s&to=%s&msg=%s&json=1" %(idsender,to,subject)
    print "https://sms.ru/sms/send?api_id=%s&to=%s&msg=%s&json=1" %(idsender,to,subject)
    res = urlopen(url)
    return  res
