extern Mailbox_Handle mbxHandleRFtx;
extern Mailbox_Handle mbxHandleUARTtx;
MsgObj msg_toUART;

uint8_t Dropout[(NUMBER_OF_DEVICES+1)];//для отсева принятых повторных пакетов на Сервере

///////////////////////////////////////////

void Preset_protocol(void)
{
 uint8_t i1;

 Subscriber.Start=START_BYTE;
 Subscriber.Number_Module=0;
 Subscriber.Role=ROLE_DEVICES;
 Subscriber.TTL=1;
 Subscriber.Repeat=REPEAT_NU;
 Subscriber.Sigma_send=SIGMA_SEND_NU;
 Statistics.Stage=STAGE_IDLE;
 Statistics.Level_now=0;
 Statistics.Number_Queue=0;
 Statistics.Router_Down=0;
 Statistics.Router_Down_Counter_Trust=0;

 for(i1=0; i1<NUMBER_OF_DEVICES; i1++)
 {
  Statistics.Status_devices_in_level_1[i1]=0;
  Statistics.List_devices_in_level_1[i1]=0;
 }

 for(i1=0; i1<MAX_SIZE_HANDOVER_LIST; i1++)
 {
  Handover[i1].Router_Down=0;
  Handover[i1].Counter=0;
 }

 indxPctBuf=0;
 indx_last_packet=0;
 NumberPacket=0;
 time_now=0;
 CounterTrust=0;

 if(Subscriber.Number_Module==1)
 {
  //RF сервер. Начало работы протокола маршрутизации
  Statistics.Stage=STAGE_SURVEY_DEVICES_IN_LEVEL_PLUS;
  Statistics.Router_Down=1;
 }
}

void Package_processing(uint8_t* Received_packet, uint8_t len, int8_t last_rssi)
{
 //Принят новый пакт
 uint8_t prb, indxCounter0, indxLevel;
 pHPacket pPacketReceiv;
 pHPacket pBuf;

 PacketReceiv.len=len;
 memcpy(PacketReceiv.buf, Received_packet, PacketReceiv.len);
 pPacketReceiv=(pHPacket)(PacketReceiv.buf);

 if((pPacketReceiv->Number_Source!=0) && (pPacketReceiv->Number_Destination!=0) && (Subscriber.Number_Module))
 {
  prb=0;
  if(Subscriber.Number_Module!=1)
  {
   if(pPacketReceiv->Type_Comand==TYPE_COMAND_ARCHIVE)
   {
    while(prb<SIZE_PCTBUF)
    {
     pBuf=(pHPacket)&PacketBuf[prb].buf;
     if((pPacketReceiv->Number_Source==pBuf->Number_Source) && (pPacketReceiv->N_Paketa==pBuf->N_Paketa))
     {
      switch (pPacketReceiv->Type_Comand)
      {
       case TYPE_COMAND_ARCHIVE:
          if(pBuf->Type_Comand==TYPE_COMAND_ARCHIVE)
          {
           if(PacketBuf[prb].repeat!=Subscriber.Repeat)
           {
            PacketBuf[prb].repeat=0;
           }
           prb=SIZE_PCTBUF;//такой пакет уже приходил
          }
       break;
      }
     }
     prb++;
    }
   }
  }

  if(prb!=(SIZE_PCTBUF+1))
  {
   //Пришел новый пакет
   if(pPacketReceiv->TTL!=0) pPacketReceiv->TTL--;

   switch(pPacketReceiv->Type_Comand)
   {
    //Получена команда опроса устройств
    case TYPE_COMAND_SURVEY_DEVICES_IN_LEVEL_PLUS:
      if(Subscriber.Number_Module!=1)
      {
       if((pPacketReceiv->Number_Destination==Subscriber.Number_Module) && (pPacketReceiv->Number_Source==Statistics.Router_Down))
       {
        Statistics.Stage=STAGE_SURVEY_DEVICES_IN_LEVEL_PLUS;
        Statistics.Router_Down_Counter_Trust=LIMIT_HANDOVER_TRUST;
       
        if(PacketReceiv.len>(1+sizeof(HPacket)))
        {
         DataUp.len=PacketReceiv.len-1-sizeof(HPacket);//ретранслируем польз. данные даже если они уже раньше отправлялись

         memcpy(DataUp.buf, &PacketReceiv.buf[sizeof(HPacket)], DataUp.len);
         DataUp.repeat=1;//если поставить 3, то будет лавинное размножение пакетов верхними модулями

         if(Dropout[0]!=DataUp.buf[DataUp.len-1])
         {
          //такие польз. данные еще в uart не отправляли. Номера сессий не совпадают
          msg_toUART.len=DataUp.len-1;
          Dropout[0]=DataUp.buf[msg_toUART.len];//фиксируем текущий номер сессии от Сервера
          memcpy(msg_toUART.buf, DataUp.buf, msg_toUART.len);
          Mailbox_post(mbxHandleUARTtx, &msg_toUART, BIOS_NO_WAIT);//передаем в uart пользовательские данные
         }
        }
       }
       else
       {
        //нижнее устройство обо мне не знает
        if(pPacketReceiv->Number_Destination==BROADCAST_ADDRESS)
        {
         if((Statistics.Router_Down==0) || ((Statistics.Router_Down==pPacketReceiv->Number_Source) && (Statistics.Router_Down_Counter_Trust==0)))
         {
          //Узел еще не опрашивали
          Statistics.Stage=STAGE_IDLE;
          Statistics.Level_now=pPacketReceiv->Level+1;
          Statistics.Router_Down=pPacketReceiv->Number_Source;

          //формируем заголовок в буфере пакета
          pBuf=(pHPacket)&PacketBuf[INDX_PCT_BUF_0].buf;
          pBuf->Start=START_BYTE;
          pBuf->Number_Source=Subscriber.Number_Module;
          pBuf->Number_Destination=pPacketReceiv->Number_Source;
          pBuf->Level=Statistics.Level_now;
          pBuf->N_Paketa=NumberPacket; NumberPacket++;
          pBuf->Status=0;
          pBuf->TTL=1;
          pBuf->Type_Comand=TYPE_COMAND_I_AM_NEW_DEVICES_IN_LEVEL_PLUS;

          PacketBuf[INDX_PCT_BUF_0].repeat=1;
          PacketBuf[INDX_PCT_BUF_0].len=sizeof(HPacket)+1;
          PacketBuf[INDX_PCT_BUF_0].buf[(PacketBuf[INDX_PCT_BUF_0].len-1)]=STOP_BYTE;
          PacketBuf[INDX_PCT_BUF_0].time_delta_for_send=(Subscriber.Sigma_send & rand());
          PacketBuf[INDX_PCT_BUF_0].time_tmp=0;
         }

         indxCounter0=MAX_SIZE_HANDOVER_LIST;
         indxLevel=MAX_SIZE_HANDOVER_LIST;
         prb=0;
         while(prb<MAX_SIZE_HANDOVER_LIST)
         {
          if(Handover[prb].Counter==0) indxCounter0=prb;//свободная ячейка
          else
          {
           if(Handover[prb].Level>pPacketReceiv->Level) indxLevel=prb;
          }
          if(Handover[prb].Router_Down==pPacketReceiv->Number_Source)
          {
           //такой роутер снизу данный узел уже искал. Его индекс заносим в indxCounter0
           indxCounter0=prb;
           prb=MAX_SIZE_HANDOVER_LIST;
          }
          prb++;
         }
         prb=0;
         if(indxCounter0<MAX_SIZE_HANDOVER_LIST)
         {
          prb=indxCounter0;
         }
         else
         {
          if(indxLevel<MAX_SIZE_HANDOVER_LIST) prb=indxLevel;
         }
         //Добавляем или обновляем таблицу handover
         if(Handover[prb].Counter<LIMIT_HANDOVER_TRUST) Handover[prb].Counter++;
         Handover[prb].Router_Down=pPacketReceiv->Number_Source;
         Handover[prb].Level=pPacketReceiv->Level;
        }
       }
      }

      if(Statistics.Stage==STAGE_RECEIVING_ANSWER_SURVEY_DEVICE_IN_LEVEL_PLUS)
      {
       //Для продления времени простоя, если была реакция верхнего устройства на команду опроса от меня
       if(Statistics.Number_Queue<NUMBER_OF_DEVICES)
       {
        Statistics.Status_devices_in_level_1[Statistics.Number_Queue]=1;
        prb=Statistics.Number_Queue;
       }
       else prb=0;
       if(pPacketReceiv->Number_Source==Statistics.List_devices_in_level_1[prb])
       {
        Statistics.time_Stage=0;//отсчет времени для Statistics.Stage
        Statistics.time_Stage_limit=SIGMA_WAIT_2s;
        if(Subscriber.Number_Module==1) Statistics.time_Stage_limit=4000;
        }
       }
    break;

    //Получен ответ от новых устройств
    case TYPE_COMAND_I_AM_NEW_DEVICES_IN_LEVEL_PLUS:
       if((pPacketReceiv->Number_Destination==Subscriber.Number_Module) && ((Statistics.Level_now+1)==pPacketReceiv->Level))
       {
        //реакция на принятый пакет TYPE_COMAND_I_AM_NEW_DEVICES_IN_LEVEL_PLUS
        prb=0;
        while(prb<NUMBER_OF_DEVICES)
        {
         if(Statistics.List_devices_in_level_1[prb]==pPacketReceiv->Number_Source)
         {
          Statistics.Status_devices_in_level_1[prb]=1;
          prb=NUMBER_OF_DEVICES;
         }
         prb++;
        }
        if(prb==NUMBER_OF_DEVICES)
        {
         //это новое устройство, которого в буфере нет
         prb=0;
         while(prb<NUMBER_OF_DEVICES)
         {
          if(Statistics.Status_devices_in_level_1[prb]==0)
          {
           Statistics.List_devices_in_level_1[prb]=pPacketReceiv->Number_Source;
           Statistics.Status_devices_in_level_1[prb]=1;
           prb=NUMBER_OF_DEVICES;
          }
          prb++;
         }
        }
        Statistics.Number_Queue=0;
       }
    break;

    //Получены измерительные данные от устройства
    case TYPE_COMAND_ARCHIVE:
       if(pPacketReceiv->Number_Destination==Subscriber.Number_Module)
       {
        //Проверяем, последний ли это пакет
        if((pPacketReceiv->Status&TYPE_PACKET_NULL)==TYPE_PACKET_NULL)
        {
         //это последний пакет архива
         if(Statistics.Number_Queue<NUMBER_OF_DEVICES)
         {
          Statistics.Status_devices_in_level_1[Statistics.Number_Queue]=1;
         }
         Statistics.Number_Queue++;//смещение для поиска нового устройства
         Statistics.Stage=STAGE_SURVEY_DEVICES_IN_LEVEL_PLUS;
        }
        pPacketReceiv->Status&=~TYPE_PACKET_NULL;

        //ищем свободную ячейку
        prb=0;
        while(prb<(SIZE_PCTBUF-1))
        {
         if(PacketBuf[prb].repeat==0)
         {
          indxPctBuf=prb;//свободная ячейка, иначе следующая по возрастанию
          prb=SIZE_PCTBUF;
         }
         prb++;
        }

        //помещаем в буфер пакетов для подтверждения
        pPacketReceiv->Number_Destination=Statistics.Router_Down;
        pPacketReceiv->TTL=1;
        pBuf=(pHPacket)&PacketBuf[indxPctBuf].buf;
        memcpy(PacketBuf[indxPctBuf].buf, PacketReceiv.buf, PacketReceiv.len);//Сохраняем пакет в буфере и разрешаем его передачу
        PacketBuf[indxPctBuf].repeat=1;
        PacketBuf[indxPctBuf].len=PacketReceiv.len;
        PacketBuf[indxPctBuf].time_delta_for_send=0;
        PacketBuf[indxPctBuf].time_tmp=0;

        if(Subscriber.Number_Module==1) PacketBuf[indxPctBuf].repeat=1;
        else PacketBuf[indxPctBuf].repeat=Subscriber.Repeat;

        indxPctBuf++;
        if(indxPctBuf>=(SIZE_PCTBUF-1)) indxPctBuf=0;

       }
    break;
   }
  }
 }
}

void Procedure_sending_packages(uint8_t** pBuf_sending_packages, uint8_t* pLen_sending_packages)
{
 //Отправка пакета
 pHPacket pBuf;
 pHPacket pBuf_from_UART;

 uint8_t i1, i2;

 time_now++;
 Statistics.time_Stage++;
 CounterTrust++;

 switch (Statistics.Stage)
 {
  //Состояние опроса или поиска новых устройств
  case STAGE_SURVEY_DEVICES_IN_LEVEL_PLUS:
      pBuf=(pHPacket)&PacketBuf[INDX_PCT_BUF_0].buf;

      pBuf->Start=START_BYTE;
      pBuf->Type_Comand=TYPE_COMAND_SURVEY_DEVICES_IN_LEVEL_PLUS;
      pBuf->Number_Source=Subscriber.Number_Module;
      i1=Statistics.Number_Queue;
      i2=NUMBER_OF_DEVICES;
      while(i1<NUMBER_OF_DEVICES)
      {
       if(Statistics.Status_devices_in_level_1[i1]!=0)
       {
        //Опрос очередного устройства
        Statistics.Number_Queue=i1;
        i2=i1;
        i1=NUMBER_OF_DEVICES;
        Statistics.Status_devices_in_level_1[Statistics.Number_Queue]++;
        pBuf->Number_Destination=Statistics.List_devices_in_level_1[Statistics.Number_Queue];
        Statistics.Stage=STAGE_RECEIVING_ANSWER_SURVEY_DEVICE_IN_LEVEL_PLUS;
        Statistics.time_Stage=0;
        Statistics.time_Stage_limit=SIGMA_WAIT_ANSWER_SURVEY_DEVICES_IN_LEVEL_PLUS;
       }
       i1++;
      }
      if(i2==NUMBER_OF_DEVICES)
      {
       //Опрос известных устройств закончен переходим к поиску новых
       pBuf->Number_Destination=BROADCAST_ADDRESS;
       Statistics.Stage=STAGE_RECEIVING_MESSAGES_FROM_NEW_DEVISES_LEVEL_PLUS;
       Statistics.time_Stage=0;
       Statistics.time_Stage_limit=2+Subscriber.Sigma_send;
      }

      pBuf->Level=Statistics.Level_now;
      pBuf->N_Paketa=NumberPacket; NumberPacket++;
      pBuf->Status=0;
      pBuf->TTL=1;

      PacketBuf[INDX_PCT_BUF_0].repeat=1;
      PacketBuf[INDX_PCT_BUF_0].len=sizeof(HPacket)+1;
      PacketBuf[INDX_PCT_BUF_0].buf[(PacketBuf[INDX_PCT_BUF_0].len-1)]=STOP_BYTE;
      PacketBuf[INDX_PCT_BUF_0].time_delta_for_send=0;
      PacketBuf[INDX_PCT_BUF_0].time_tmp=0;
  break;

  //Состояние приема ответов от новых устройств в Level+1
  case STAGE_RECEIVING_MESSAGES_FROM_NEW_DEVISES_LEVEL_PLUS:
      if(Statistics.time_Stage>Statistics.time_Stage_limit)
      {
       if(Subscriber.Number_Module==1)
       {
        //Разрешаем выводить архив в целях ретрансляции
        Statistics.Stage=STAGE_PROCESSING_ARCHIVE;
        Statistics.counter_Stage_Processing_Archive=SIZE_PCTBUF-1;//MAX_PACKET_SEND_PROCESSING_ARCHIVE;
        Statistics.Number_Queue=0;//для перехода к исходному устройству для опроса
       }
       else
       {
        //Разрешаем выводить архив
        Statistics.Stage=STAGE_PROCESSING_ARCHIVE;
        Statistics.counter_Stage_Processing_Archive=MAX_PACKET_SEND_PROCESSING_ARCHIVE;
        Statistics.Number_Queue=0;

        //Закончился этап поиска новых устройств в Level+1, теперь выводим архив, после него
        //отправляем Нуль пакет, чтобы нижний роутер начал опрашивать следующее устройство
        pBuf=(pHPacket)&PacketBuf[INDX_PCT_BUF_0].buf;
        pBuf->Start=START_BYTE;
        pBuf->Type_Comand=TYPE_COMAND_ARCHIVE;
        pBuf->Number_Source=0;
        pBuf->Number_Destination=Statistics.Router_Down;
        pBuf->Level=Statistics.Level_now;
        pBuf->N_Paketa=NumberPacket; NumberPacket++;
        pBuf->Status=0;
        pBuf->Status|=TYPE_PACKET_NULL;
        pBuf->TTL=1;

        PacketBuf[INDX_PCT_BUF_0].repeat=1;
        PacketBuf[INDX_PCT_BUF_0].len=sizeof(HPacket)+1;
        PacketBuf[INDX_PCT_BUF_0].buf[(PacketBuf[INDX_PCT_BUF_0].len-1)]=STOP_BYTE;
        PacketBuf[INDX_PCT_BUF_0].time_delta_for_send=0;
        PacketBuf[INDX_PCT_BUF_0].time_tmp=0;
       }
      }
  break;

  case STAGE_RECEIVING_ANSWER_SURVEY_DEVICE_IN_LEVEL_PLUS:
      if(Statistics.time_Stage>Statistics.time_Stage_limit)
      {
       //Время закончилось, а ответа нет. Переходим к опросу следующего устройства
       if(Statistics.Number_Queue<NUMBER_OF_DEVICES)
       {
        if(Statistics.Status_devices_in_level_1[Statistics.Number_Queue]>LIMIT_TRUST) Statistics.Status_devices_in_level_1[Statistics.Number_Queue]=0;
       }
       Statistics.Number_Queue++;//смещение для поиска нового устройства
       Statistics.Stage=STAGE_SURVEY_DEVICES_IN_LEVEL_PLUS;
      }
  break;
 }

 if(Mailbox_pend(mbxHandleRFtx, &msg_uart, BIOS_NO_WAIT))
 {
  //заполняем буфер измерительными данными из uart
  if((Subscriber.Number_Module!=1) && (msg_uart.len<PACKET_SIZE_MAX))
  {
   //ищем свободную ячейку
   i1=0;
   while(i1<(SIZE_PCTBUF-1))
   {
    if(PacketBuf[i1].repeat==0)
    {
     indxPctBuf=i1;//свободная ячейка, иначе следующая по возрастанию
     i1=SIZE_PCTBUF;
    }
    i1++;
   }

   //формируем пакет Архива в буфере пакетов. Если это не делать, то информация с устройств будет идти только в момент его опроса,
   if(PacketBuf[INDX_PCT_BUF_0].len==(sizeof(HPacket)+1))
   {
    if(msg_uart.len>sizeof(HPacket))
    {
     //добавляем к нуль пакету полезные данные
     pBuf=(pHPacket)&PacketBuf[INDX_PCT_BUF_0].buf;
     memcpy(&PacketBuf[INDX_PCT_BUF_0].buf[sizeof(HPacket)], &msg_uart.buf[sizeof(HPacket)], (msg_uart.len-sizeof(HPacket)));
     PacketBuf[INDX_PCT_BUF_0].len=msg_uart.len;
    }
   }
   else
   {
    if(msg_uart.len>sizeof(HPacket))
    {
     pBuf=(pHPacket)&PacketBuf[indxPctBuf].buf;
     pBuf->Start=START_BYTE;
     pBuf->Type_Comand=TYPE_COMAND_ARCHIVE;
     pBuf->Number_Source=0;
     pBuf->Number_Destination=Statistics.Router_Down;
     pBuf->Level=Statistics.Level_now;
     pBuf->N_Paketa=NumberPacket; NumberPacket++;
     pBuf->Status=0;
     pBuf->TTL=1;

     memcpy(&PacketBuf[indxPctBuf].buf[sizeof(HPacket)], &msg_uart.buf[sizeof(HPacket)], (msg_uart.len-sizeof(HPacket)));

     PacketBuf[indxPctBuf].repeat=Subscriber.Repeat;
     PacketBuf[indxPctBuf].len=msg_uart.len;
     PacketBuf[indxPctBuf].time_delta_for_send=0;
     PacketBuf[indxPctBuf].time_tmp=1;
     indxPctBuf++;
     if(indxPctBuf>=(SIZE_PCTBUF-1)) indxPctBuf=0;
    }
   }
   pBuf_from_UART=(pHPacket)&msg_uart.buf;
   if(Subscriber.Number_Module!=pBuf_from_UART->Number_Source) srand(pBuf_from_UART->Number_Source);
   Subscriber.Number_Module=pBuf_from_UART->Number_Source;
   pBuf->Number_Source=Subscriber.Number_Module;
   pBuf->Status|=pBuf_from_UART->Status;
  }
 }

 //Прошло время доверия
 if(CounterTrust>4001)
 {
  CounterTrust=0;
  if(Statistics.Router_Down_Counter_Trust) Statistics.Router_Down_Counter_Trust--;//гистерезис
  {
   //доверие к нижнему роутеру закончилось, он меня не опрашивал более 3 с
   if(Statistics.Router_Down_Counter_Trust && Statistics.Level_now) i2=Statistics.Level_now-1;//Связь с нижним роутером не потеряна, поэтому будем переключаться если только есть надежный нжний роутером с более меньшит Level
   else i2=0xFF;
   for(i1=0; i1<MAX_SIZE_HANDOVER_LIST; i1++)
   {
    if(Handover[i1].Counter>(LIMIT_HANDOVER_TRUST-1))
    {
     if(Handover[i1].Level<i2)
     {
      i2=Handover[i1].Level;
      Statistics.Router_Down=Handover[i1].Router_Down;
      Statistics.Router_Down_Counter_Trust=0;
     }
    }
   }
  }

  for(i1=0; i1<MAX_SIZE_HANDOVER_LIST; i1++)
  {
   //искуственно уменьшаем доверие к роутерам из списка handover, это позволи удалить неактивные роутеры,
   if(Handover[i1].Counter) Handover[i1].Counter--;
  }
 }

 for(i1=0; i1<SIZE_PCTBUF; i1++) PacketBuf[i1].time_tmp++;//приращение времени в ячейках буфера передачи

 if(Statistics.Stage==STAGE_PROCESSING_ARCHIVE)
 {
  if(Statistics.counter_Stage_Processing_Archive)
  {
   i1=indx_last_packet;//разрешаем выводить пакеты из буфера Архива начиная с последнего индекса
   Statistics.counter_Stage_Processing_Archive--;
  }
  else
  {
   //закончился вывод разрешенного количества пакетов из Архива
   if(Subscriber.Number_Module==1)
   {
    Statistics.Stage=STAGE_SURVEY_DEVICES_IN_LEVEL_PLUS;
    i1=SIZE_PCTBUF;//ничего не отправляем
   }
   else i1=SIZE_PCTBUF-1;//отправляем нуль пакет.
  }
 }
 else i1=SIZE_PCTBUF-1;//отправляем только команды, в том числе нуль пакет.

 while(i1<SIZE_PCTBUF)//отправка по текущего пакета
 {
  pBuf=(pHPacket)&PacketBuf[i1].buf;
  if((PacketBuf[i1].repeat) && (pBuf->TTL))
  {
   if(PacketBuf[i1].time_tmp>=PacketBuf[i1].time_delta_for_send)
   {
    PacketBuf[i1].repeat--;
    if(pBuf->Type_Comand==TYPE_COMAND_ARCHIVE)
    {
     time_now+=MULTI_K;//учет длинны пакета АРХИВ, который передаетсяв течение нескольких квантов
     PacketBuf[i1].time_delta_for_send=PacketBuf[i1].time_tmp+MULTI_K*MAX_PACKET_SEND_PROCESSING_ARCHIVE*2;//чтобы сделать паузу для ожидания копий от нижнего роутера, а не отправлять сразу два пакета в одином цикле опроса
    }
    //Подменяем в пакетах Архива адрес назначения на текущий, т.к. данные в архиве лежат давно, а номер нижнего роутера мог поменяться или быть равным нулю (у своих пакетов при включении устройства)
    if((Statistics.Stage==STAGE_PROCESSING_ARCHIVE) && (i1!=(SIZE_PCTBUF-1))) pBuf->Number_Destination=Statistics.Router_Down;
    if((Statistics.Stage==STAGE_PROCESSING_ARCHIVE) && (i1==(SIZE_PCTBUF-1)))
    {
     Statistics.Stage=STAGE_IDLE;
    }
    indx_last_packet=i1+1;//для равномерного вычитывания архива
    if(indx_last_packet>=(SIZE_PCTBUF-1)) indx_last_packet=0;
 
    *pBuf_sending_packages=PacketBuf[i1].buf; //пакет на отправку
    if(Subscriber.Number_Module) *pLen_sending_packages=PacketBuf[i1].len;
    i1=SIZE_PCTBUF;
   }
  }
  i1++;
 }
 if(i1!=(1+SIZE_PCTBUF)) *pLen_sending_packages=0;
}
