
chSplit1 = ':'
chSplit2 = ','
chSplit3 = '`'

def GetKey(data, separator):
    return GetIndexStr(data, separator, 0)

def GetVal(data, separator):
    return GetIndexStr(data, separator, 1)

def GetIndexStr(data, separator, index):

    stringData = 0     #variable to count data part nr
    dataPart = ""      #variable to hole the return text
    print(f'data:{data}, separator:{separator}, index:{index}')
    for i in  data: # Walk through the text one letter at a time
        
        if i == separator:
            print(f'{i}')
        # #            Count the number of times separator character appears in the text
            stringData += 1
            print(f'{stringData}')
        
        elif stringData == index:
            #get the text when separator is the rignt one
            dataPart.append(i)
        elif stringData > index:
            #return text and stop if the next separator appears - to save CPU-time
            print(f'dataPart: {dataPart}')
            return dataPart
            break
    
    #return text if this is the last part
    return dataPart

def GetParseData(data):
    parsed_data = {}
    pairs = data.split(chSplit3)

    for pair in pairs:
        if pair:
            key, value = pair.split(chSplit1)
            parsed_data[key] = value
    return parsed_data