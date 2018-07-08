### DVB-T-Extractor
Simple C++ application for DVB-T parsing/demultiplexing. I implemented this application as part of course [BMS (Wireless and Mobile Networks)](http://www.fit.vutbr.cz/study/courses/BMS/index.php.en) at FIT BUT.

#### Usage  
`./bms2 <file_name>.ts`

#### Program Features:
Program retrieves an input file (`file_name.ts`) that contains the transport data of the DVB-T broadcast stream. The output of the application is a file called `filename.txt`, which contains details for each extracted channel from the processed transport stream.

#### Output properties
- The file contains information about the processed multiplex obtained from the NIT service table in the header, details about the individual programs obtained from the PAT and SDT tables, which are supplemented by the aggregated bit rate statistic for all channels belonging to one program.
- Each line describing one multiplex program has the following format:  
`PID-service_provider-service_name: <bitrate> Mbps`
- The individual rows are sorted by the PID value.
- Bitrate count based on the number of packets of the given program relative to the total number of packets according to the formula:  
`program_bitrate = packet_pack_program / total_package_package * bitrate_stream`
- If the program contains multiple video / audio tracks or service information, bitrate of all these channels is read into a common value.
- Transfer speed is rounded to 2 decimal places.  

#### Output Example:
```
              Network name: SIT 1 CESKA TELEVIZE
              Network ID: 12345
              Bandwidth: 8 MHz
              Constellation: 64-QAM
              Guard interval: 1/4
              Code rate: 2/3
              
              0x0100-Ceska televize-CT 1 JM: 10.50 Mbps
              ...
```
#### Useful links
 [ Example .ts file](https://www.fit.vutbr.cz/study/courses/BMS/public/proj2016/multiplex.zip)  
 [ DVB standard ETSI EN 300 468 V1.13.1](https://www.etsi.org/deliver/etsi_en/300400_300499/300468/01.13.01_40/en_300468v011301o.pdf)
