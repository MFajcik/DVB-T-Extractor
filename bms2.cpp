/**
*  Author: Bc. Martin Fajcik
*  Institution: FIT@VUT
*  Author's ID (login): xfajci00
*  Course: BMS - Wireless and Mobile Networks
*  Project No.: 2
*  Last modified: 12.12.2016
*/

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include  <iomanip>
#include <string>
#include <stdlib.h>
#include <algorithm> 

#include <vector>
#include <map>
#include <fstream>
#include <sstream>

#include <assert.h>

#define DEBUG false

const uint32_t MPEG_PAYLOAD_SIZE = 184;
const uint32_t MPEG_HEADER_SIZE = 4;
const uint32_t MPEG_PACKET_SIZE = MPEG_HEADER_SIZE + MPEG_PAYLOAD_SIZE;

const uint8_t TABID_PROGRAM_ASSOCIATION_SECTION = 0x00;
const uint8_t TABID_PROGRAM_MAP_SECTION = 0x02;
const uint8_t TABID_NWINFOSEC_ACTUALNETWORK = 0x40;
const uint8_t TABID_SWDESCSEC_ACTUALTS = 0x42;

const uint32_t NIT_PID = 0x0010; 
const uint32_t PAT_PID = 0x0000; 
const uint32_t SDT_PID = 0x0011; 

//Descriptor Tags
const uint8_t TAG_network_name_descriptor = 0x40;
const uint8_t TAG_terrestrial_delivery_system_descriptor = 0x5A;
const uint8_t TAG_service_descriptor = 0x48;

using namespace std;

class TS_NITInfo{
    uint16_t ts_id;
    string *bandwidth, *constellation, *guard_interval, *code_rate;
    
public:
    TS_NITInfo(uint16_t id, string *bw, string *cst, string *gi, string *cr )
    :ts_id(id), bandwidth(bw), constellation(cst), guard_interval(gi), code_rate(cr)
    {}
    
    
    string   getBandwidth()    { return (bandwidth)?*bandwidth:string();           }
    string   getConstellation(){ return (constellation)?*constellation:string();   }
    string   getGuardInterval(){ return (guard_interval)?*guard_interval:string(); }
    string   getCodeRate()     { return (code_rate)?*code_rate:string();           }
    uint16_t getTSID()         { return ts_id;                                     }
    
    ~TS_NITInfo(){      
        if (bandwidth)
            delete bandwidth;
        if (constellation)
            delete constellation;
        if (guard_interval)
            delete guard_interval;
        if (code_rate)
            delete code_rate;
    }
};

/*   For testing purposes only     */
#if DEBUG
    #define dumpRawPacket(x)  dmpRawPacket(x)
    void dmpRawPacket(vector<uint8_t>& packet) {
        for (int i = 0; i<packet.size(); i++) {
            cout << setfill('0') << setw(2) << hex << static_cast<int>(packet[i]) << dec;
        }
        cout << "\n";
    }


    #define dumpHexaByte(x)  dmpHexaByte(x)
    void dmpHexaByte(uint8_t byte) {
        cout << setfill('0') << setw(2) << hex << static_cast<int>(byte) << dec << "\n";
    }
    
    #define dumpTsNitInfo(x)  dmpTsNitInfo(x)
    void dmpTsNitInfo(TS_NITInfo* _i){
        TS_NITInfo info = *_i;
        cout<<"--------------TSNITINFODUMP------------ \n";
        cout<<"TS: "<<dec<<info.getTSID()<<"\n";
        cout<<"BW: "<<info.getBandwidth()<<"\n";
        cout<<"CS: "<<info.getConstellation()<<"\n";
        cout<<"GI: "<<info.getGuardInterval()<<"\n";
        cout<<"CR: "<<info.getCodeRate()<<"\n";
        cout<<"--------------------------------------- \n";
    }

    #define d_assert(x) assert(x)

#else
    #define dumpRawPacket(x)
    #define dumpHexaByte(x) 
    #define d_assert(x)    
    #define dumpTsNitInfo(x)
#endif

const uint32_t AFC_PAYONLY = 0x01;
const uint32_t AFC_AFCONLY = 0x02;
const uint32_t AFC_AFCANDPAYLOAD = 0x03;

const uint32_t SCR_NOTSCR = 0x0;

#define PRINT_ERR_WITH_PID(pid,a) (cerr<<"PID: "<<(pid)<<" "<<(a)<<"\n")

enum class PType {
	NIT,
	AFCONLY,
	PAT,
	PMT,
    SDT,
	UNKNOWN
};

typedef struct PCR_t{
    uint64_t base,extension;
    PCR_t(){ base = 0; extension = 0; };
    PCR_t( uint64_t _b, uint64_t _e ){
        base = _b;
        extension = _e;
    }
    uint64_t calculatePCR(){return (base*300)+extension; }
}PCR_t;

class MPEGPacket {
private:
	vector<uint8_t> packet;
	uint32_t header,
		TEI, //Set when a demodulator can't correct errors from FEC data; indicating the packet is corrupt.
		PUSI, //Set when a PES, PSI, or DVB-MIP packet begins immediately following the header.
		TransportPriority, //Set when the current packet has a higher priority than other packets with the same PID.
		PID, // Packet Identifier, describing the payload data.
		TSC,
		AdaptationFieldControl,
		pointer_field, //Sections may start at the beginning of the payload of a TS packet,
					   //but this is not a requirement, because the start of the first section in the payload of a TS packet is pointed to by the
					   //pointer_field
		ContinuityCounter; //Sequence number of payload packets (0x00 to 0x0F) within each stream (except PID 8191)
						   //Incremented per-PID, only when a payload flag is set.
	vector<uint8_t> payload;
	PType type;
    unsigned payloadoffset;
    bool ispayloaddirty;
    uint64_t packet_number;
    PCR_t PCR;

void setPacketNumber(){
    static uint64_t counter=0;
    packet_number=counter;
    counter++;
}
    
public:
	MPEGPacket(vector<uint8_t> _p) : packet(MPEG_PACKET_SIZE, 0), type(PType::UNKNOWN), ispayloaddirty(false) {
		this->packet = _p;
		header = 0x0;
        setPacketNumber();
		//read head
		for (int i = 0; i<MPEG_HEADER_SIZE; i++) {
			header <<= 8;
			header |= packet[i];
		}
		//check the packet head
		d_assert(packet[0] == 0x47);

		this->TEI = (header & 0x800000) >> 23;
		this->PUSI = (header & 0x400000) >> 22;
		this->TransportPriority = (header & 0x200000) >> 21;
		this->PID = (header & 0x1fff00) >> 8;
		this->TSC = (header & 0xc0) >> 6;
		this->AdaptationFieldControl = (header & 0x30) >> 4;
		this->ContinuityCounter = (header & 0xf);
        
		this->type = PType::UNKNOWN; 
        if (TEI)
			PRINT_ERR_WITH_PID(PID, "Detected corrupted packet.");

        //With the exception of the EIT carrying schedule information, 
        //all tables specified in the present document shall not be scrambled.
		//if (TSC)
		//	PRINT_ERR_WITH_PID(PID, "Scrambled channels are not supported.");

		switch (AdaptationFieldControl) {
		case AFC_PAYONLY:
            if (PUSI)
                this->pointer_field = packet[4];
            else
                this->pointer_field = 0x0;
            
			//read payload
            this->payloadoffset = MPEG_HEADER_SIZE+PUSI+pointer_field;
			for (int i = payloadoffset; i<MPEG_PACKET_SIZE; i++)
				payload.push_back(packet[i]);
			break;
		case AFC_AFCONLY:
			this->type = PType::AFCONLY;   
            if (containsPCR())        
                this->PCR=readPCR();        
			break;
		case AFC_AFCANDPAYLOAD:{          
            if (containsPCR())       
                this->PCR=readPCR();  
            
			uint8_t AFLength = packet[4];
            if (PUSI)
                this->pointer_field = packet[4 + 1 + AFLength];//behind AF field 
            else
                this->pointer_field = 0x0;  
            this->payloadoffset = MPEG_HEADER_SIZE + PUSI + AFLength + 1 + pointer_field;
			for (int i = payloadoffset ; i<MPEG_PACKET_SIZE; i++) //+1 for AF Field Length byte, +1 because we do not want pointer_field in payload
				payload.push_back(packet[i]);
			break;
        }
        default:
            cerr<<"Unknown AdaptationFieldControl flag value\n";
            break;
		}
	}

	PType getPacketType() {
		return this->type;
	}

	vector<uint8_t> &getPayload() { return this->payload; }
	uint32_t getHeader()          { return this->header;  }
	uint16_t getPID()             { return this->PID;     }
    uint32_t getPUSI()            { return this->PUSI;    }
	uint32_t getCC()              { return this->ContinuityCounter; }
    uint64_t getPacketNo()        { return this->packet_number; }
    PCR_t    getPCR()             { return this->PCR;     }
    
	vector<uint8_t>&getPacket(){ 
        if (ispayloaddirty) updatePayload(); 
        return this->packet;  
    }
    
    PCR_t readPCR(){
        unsigned offset = 6;
        uint64_t pcrbase = ((uint64_t) packet[offset])<<25;
        pcrbase |= ((uint64_t) packet[offset+1])<<17;
        pcrbase |= ((uint64_t) packet[offset+2])<<9;
        pcrbase |= ((uint64_t) packet[offset+3])<<1;
        pcrbase |= ((uint64_t) (0x80&packet[offset+4]))>>7;
        
        uint64_t pcr_extension = ((uint64_t)0x01 & packet[offset+4] )<<8;
        pcr_extension |= packet[offset+5];
        return PCR_t(pcrbase,pcr_extension);
    }
    bool containsAdaptationField(){
        return AdaptationFieldControl==AFC_AFCONLY ||
        AdaptationFieldControl==AFC_AFCANDPAYLOAD;
    }
    
    bool containsPCR(){
        return packet[5]&0x10;
    }
    
    bool isDiscontunityIndicator(){
        return packet[5]&0x80;
    }
    
    void appendToPayload(vector<uint8_t>& appendablePayload){   
        payload.insert(payload.end(), appendablePayload.begin(), appendablePayload.end());
        ispayloaddirty=true;
    }
    
    void updatePayload(){
        ispayloaddirty = false;
        //lazy update
        packet.erase(packet.begin()+payloadoffset, packet.end());
        packet.insert(packet.end(), payload.begin(), payload.end());
    }
    
    size_t size(){ return this->packet.size(); }
    
    bool operator==(MPEGPacket &rhs ) {
        if (ispayloaddirty) updatePayload();
        if (packet.size() != rhs.size()) return false;
        for (int i=0; i<packet.size();i++)
            if (packet[i]!=rhs[i]) return false;
        return true;
    }
    
	uint8_t& operator[](unsigned idx) {
        if (ispayloaddirty) updatePayload();
		return this->packet[idx];
	}
    void setType(PType _type){
        this->type=_type;
    }
};

template <class T> class Tables {
	vector<T*> tables;

public:
	void add(T* i) {
		this->tables.push_back(i);
	}
    
	vector<T*>& get() {
		return tables;
	}
    
    size_t size(){
        return tables.size();
    }

	T& operator[](unsigned idx) {
		return *(this->tables[idx]);
	}
	const T& operator[](unsigned idx) const {
		return *(this->tables[idx]);
	}
	Tables() {}
	
	void cleanup() {
		for (auto table : tables)
           delete table;
        tables.clear();
	}
};

class Table{
protected:
    MPEGPacket* packet;
    vector<uint8_t>payload;
    uint8_t table_id,section_number, last_section_number;
    uint16_t ts_id;
    
    Table(MPEGPacket* _p){
        this->packet = _p;
        this->payload = packet->getPayload();
        this->table_id = payload[0];
          
    }
    //Iterate over descriptors, find one with right tag
    vector<uint8_t>* findFirstDescForTag(uint16_t desclen, vector<uint8_t>&payload, const unsigned offset, const uint8_t TAG){
        unsigned tagindex=0;
        while (tagindex<desclen){
            if (!(tagindex+1<desclen))
                break;
            unsigned size = payload[offset+tagindex+1];;
            if (payload[offset+tagindex]==TAG){
                return new vector<uint8_t>(payload.begin()+offset+tagindex, 
                      payload.begin()+offset+tagindex+size+2);//+2 for 2 start bytes - tag and length
                break;
            }
            tagindex+=size+2; //jump to next descriptor
        }
        return nullptr;
    }
    public:
    MPEGPacket* getPacket(){
        return packet;
    }
};

class NIT: protected Table {
    uint16_t network_id;
    vector<TS_NITInfo*> tsinfos;
    string *network_name;
    const unsigned NEWDESCOFFSET = 10;
    
public:
	NIT(MPEGPacket* _p): Table(_p), network_name(nullptr){
        
        packet->setType(PType::NIT);
        d_assert(table_id == TABID_NWINFOSEC_ACTUALNETWORK);
        
        this->network_id=(((uint16_t)payload[3])<<8) | payload[4];     
        
        uint16_t nwdesclen=(((uint16_t)(payload[8]&0x0F))<<8) | payload[9];
        
        vector<uint8_t>* nwnamedesc=findFirstDescForTag(nwdesclen, payload, NEWDESCOFFSET, TAG_network_name_descriptor);
        if (nwnamedesc){
            uint8_t size = (*nwnamedesc)[1];
            network_name= new string(nwnamedesc->begin()+2, nwnamedesc->begin()+size+2);
            delete nwnamedesc;
        }
        
        
        const unsigned tsloopoffset=9+1+nwdesclen+1+1;
        uint16_t tsloopslen=(((uint16_t)(payload[tsloopoffset-2]&0x0F))<<8) | payload[tsloopoffset-1];
        unsigned sindex=0;
        while (sindex<tsloopslen){
            const unsigned tsdescoffset=tsloopoffset+sindex+6;
            uint16_t ts_id = (((uint16_t)payload[tsloopoffset])<<8)|payload[tsloopoffset+1];
            uint16_t tsdesclen=(((uint16_t)(payload[tsdescoffset-2]&0x0F))<<8) | payload[tsdescoffset-1];
            vector<uint8_t>* tsTerrDelSysDesc=findFirstDescForTag(tsdesclen, payload, tsdescoffset, TAG_terrestrial_delivery_system_descriptor);
            if (tsTerrDelSysDesc){                
                string *bandwidth, *constellation, *guard_interval, *code_rate;
                /**
                    Bandwidth encoding info
                        000 8 MHz
                        001 7 MHz
                        010 6 MHz
                        011 5 MHz
                */
                uint8_t bwbyte= (*tsTerrDelSysDesc)[6]>>5;
                switch(bwbyte){
                case 0x0:                    
                    bandwidth = new string("8 MHz");
                    break;
                case 0x1:
                    bandwidth = new string("7 MHz");
                    break;
                case 0x2:
                    bandwidth = new string("6 MHz");
                    break;
                case 0x3:
                    bandwidth = new string("5 MHz");
                    break;
                default:
                    bandwidth = nullptr;
                    break;
                }
                
                /**
                    Constellation encoding
                        00 QPSK
                        01 16-QAM
                        10 64-QAM
                */
                uint8_t conbyte=(*tsTerrDelSysDesc)[7]>>6;                
                switch(conbyte){
                case 0x0:                    
                    constellation = new string("QPSK");
                    break;
                case 0x1:
                    constellation = new string("16-QAM");
                    break;
                case 0x2:
                    constellation = new string("64-QAM");
                    break;
                default:
                    constellation = nullptr;
                    break;
                }
                /**
                    Code rate encoding
                        000 1/2
                        001 2/3
                        010 3/4
                        011 5/6
                        100 7/8
                */
                uint8_t crbyte=(*tsTerrDelSysDesc)[7]&0x07;
                switch(crbyte){
                case 0x0:                    
                    code_rate = new string("1/2");
                    break;
                case 0x1:
                    code_rate = new string("2/3");
                    break;
                case 0x2:
                    code_rate = new string("3/4");
                    break;
                case 0x3:
                    code_rate = new string("5/6");
                    break;
                case 0x4:
                    code_rate = new string("7/8");
                    break;
                default:
                    code_rate = nullptr;
                    break;
                }
                
                /** Guard interval encoding
                        00 1/32
                        01 1/16
                        10 1/8
                        11 1/4
                */
                uint8_t gibyte=(*tsTerrDelSysDesc)[8];
                gibyte=(gibyte&0x18)>>3;
                switch(gibyte){
                case 0x0:                    
                    guard_interval = new string("1/32");
                    break;
                case 0x1:
                    guard_interval = new string("1/16");
                    break;
                case 0x2:
                    guard_interval = new string("1/8");
                    break;
                case 0x3:
                    guard_interval = new string("1/4");
                    break;
                default:
                    guard_interval = nullptr;
                    break;
                }
                tsinfos.push_back(new TS_NITInfo(ts_id,bandwidth,constellation,
                    guard_interval, code_rate));
                delete tsTerrDelSysDesc;
                break;
            }
            unsigned loopsize=6+tsdesclen;
            sindex+=loopsize+2;
        }
        
	}
    
    uint16_t getNetworkId()    { return network_id; }
    string   getNetworkname()  { return (network_name)?*network_name:string();     }  
    vector<TS_NITInfo*>& getTSInfos(){return tsinfos;}
    
};

class Service{
    uint16_t ID;
    string *service_provider_name, *service_name;

public:
    Service(uint16_t id, string *spname, string *sname):
    ID(id), service_provider_name(spname),service_name(sname){};
    
    uint16_t getServiceID(){ return ID; }
    string getServiceProviderName(){ return *service_provider_name; }
    string getServiceName(){ return *service_name; }
    
    ~Service() {  
        if (service_provider_name)
            delete service_provider_name;
        if (service_name)
            delete service_name;
	}
};



class PMT : public Table {
    vector<uint16_t> componentPIDs;
    
public:
    PMT(MPEGPacket* _p):Table(_p){        
        packet->setType(PType::PMT);
        
        d_assert (table_id == TABID_PROGRAM_MAP_SECTION);
        uint16_t section_length = (((uint16_t)payload[1]&0x0F)<<8) | payload[2];
        uint16_t program_info_length = (((uint16_t)payload[10]&0xF)<<8)| payload[11];
        
        unsigned section_end = section_length+3; // payload index must not exceed this index, no valid data beyond it, payload[section_end] is invalid
        const unsigned components_loop_i_start = 12+program_info_length;;
        unsigned components_loop_i = components_loop_i_start;
        while (components_loop_i<(section_end-4)){
            uint16_t elementaryPID = (((uint16_t)payload[(components_loop_i+1)]&0x1F)<<8)| payload[(components_loop_i+2)];
            componentPIDs.push_back(elementaryPID);
            unsigned ES_info_length = (((uint16_t)payload[(components_loop_i+3)]&0x0F)<<8)| payload[(components_loop_i+4)];
            components_loop_i+=5+ES_info_length;
        }   
    }
    
    vector<uint16_t>& getComponendPIDs(){ return componentPIDs; }
};

class Program{
    uint16_t program_number, program_map_pid;

public:
    Program(uint16_t pn, uint16_t pid):
    program_number(pn),program_map_pid(pid){};
    
    uint16_t getProgramNumber(){ return program_number;  }
    uint16_t getProgramMapPID(){ return program_map_pid; }
   
}; 

class PAT : protected Table{
    uint16_t ts_id;
    vector<Program*> programs;
    map<uint16_t, PMT*> PMTs;
    
public:
    PAT(MPEGPacket* _p):Table(_p){
        packet->setType(PType::PAT);
        
        d_assert(table_id == TABID_PROGRAM_ASSOCIATION_SECTION); 
        uint16_t section_length = (((uint16_t)payload[1]&0x0F)<<8) | payload[2];
        ts_id=(((uint16_t)payload[3])<<8)|payload[4];
        unsigned section_end = section_length+3; // payload index must not exceed this index, no valid data beyond it, payload[section_end] is invalid
        const unsigned program_loop_i_start = 8;
        unsigned program_loop_i = program_loop_i_start;
        while (program_loop_i<(section_end-4)){            
            uint16_t program_number = (((uint16_t)payload[program_loop_i])<<8) | payload[program_loop_i+1];
            if (program_number){
                uint16_t program_map_pid = (((uint16_t)payload[program_loop_i+2]&0x1F)<<8) | payload[program_loop_i+3];
                programs.push_back(new Program(program_number, program_map_pid));
            }
            program_loop_i+=3+1; //3 is length of body
        }
    }
    uint16_t getTsID() { return ts_id; }
    vector<Program*>& getPrograms() { return programs; }
    
    PMT* getPMT(uint16_t programnumber, vector<MPEGPacket*>& packets){
        for (auto program: programs){
            if (program->getProgramNumber() == programnumber){
                if (PMTs.find(programnumber) != PMTs.end())
                    return PMTs[programnumber];
                else{
                    for (auto packet: packets)
                        if (packet->getPID()==program->getProgramMapPID())
                            return new PMT(packet);
                }
                break;
            }
        }
        return nullptr;
    }
    
    ~PAT(){
        for (auto const& pmt: PMTs)
            delete pmt.second;
    }
};

class SDT : public Table{
    uint16_t ts_id;
    vector<Service*> services;
    public:
    SDT(MPEGPacket* _p):Table(_p){
        packet->setType(PType::SDT);

        d_assert(table_id == TABID_SWDESCSEC_ACTUALTS);  
        uint16_t section_length = (((uint16_t)payload[1]&0x0F)<<8) | payload[2];
        ts_id=(((uint16_t)payload[3])<<8)|payload[4];
        unsigned section_end = section_length+3; // payload index must not exceed this index, no valid data beyond it, payload[section_end] is invalid
        const unsigned service_loop_i_start = 11;
        
        unsigned service_loop_i=service_loop_i_start;
        while (service_loop_i<(section_end-4)){ //-4 because CRC_32 follows
            uint16_t service_id = (((uint16_t)payload[service_loop_i])<<8)|payload[service_loop_i+1];
            uint16_t svdesclen = (((uint16_t)payload[service_loop_i+3]&0x0F)<<8)|payload[service_loop_i+4];
            const unsigned desc_i_start = service_loop_i+5;
            vector<uint8_t>* svdesc=findFirstDescForTag(svdesclen, payload, desc_i_start, TAG_service_descriptor);
            if (svdesc){
                uint8_t provider_name_length = (*svdesc)[3];
                uint8_t service_name_length = (*svdesc)[3+provider_name_length+1];
                string* provider_name = new string(svdesc->begin()+4, svdesc->begin()+provider_name_length+4);
                unsigned offset_sname = provider_name_length+5;
                string* service_name = new string(svdesc->begin()+offset_sname, svdesc->begin()+service_name_length+offset_sname);
                services.push_back(new Service(service_id, provider_name, service_name));
                delete svdesc;
            }
            service_loop_i+=4+svdesclen+1;
        }
    }
    
    uint16_t getTsID() { return ts_id; }
    vector<Service*>& getServices() { return services; }
    
};

static bool readpacket(vector<uint8_t>& rawpacket, ifstream& s) {
    vector<uint8_t> _p;
    uint8_t byte = 0x00;
    while (s.good() && byte != 0x47) {
        s.read((char*)&byte, 1);
    }
    if (!s.good()) return false;
    rawpacket[0] = 0x47;
    s.read((char*)&rawpacket[1], MPEG_PACKET_SIZE - 1);
    return (bool)s;
}


void mergePackets(vector<MPEGPacket*>& packets){
    map<uint16_t,MPEGPacket*> headPackets;
    vector<MPEGPacket*> mergedpackets;
    
    for (auto packet: packets)
    {        
        if (packet->getPUSI()){
            headPackets[packet->getPID()]=packet;
            mergedpackets.push_back(packet);
        }
        else{
            //merge payloads
            if ( headPackets.find(packet->getPID()) != headPackets.end() ) {
                MPEGPacket* firstSeqPacket= headPackets[packet->getPID()];
                if (firstSeqPacket!=nullptr)
                   firstSeqPacket->appendToPayload(packet->getPayload());
            }
        }
    }  
    packets=mergedpackets;
}

uint64_t streambitrate = 0;
const uint64_t SYS_CLOCK_F = 27000000; // see ETSI TR 101 290, p. 159
/**
    Calculated bitrate for each PID, stream bitrate is an average of each PID's bitrate
*/
void updateBitrate(MPEGPacket* packet){
    static map <uint16_t,MPEGPacket*> firstPCRPackets;
    static map <uint16_t,uint64_t> bitrates;
    if (!(packet->containsAdaptationField()) || !(packet->containsPCR()))
        return;
      
    bool firstPCRundefined = firstPCRPackets.find(packet->getPID()) == firstPCRPackets.end();
    
    MPEGPacket* p; 
    PCR_t firstPCR; 
    uint64_t firstPCRPacketNo;
    
    if (!firstPCRundefined){
        p = firstPCRPackets[packet->getPID()];
        firstPCR = p->getPCR();
        firstPCRPacketNo = p->getPacketNo();
    }
    
    PCR_t newPCR = packet->getPCR();
    uint64_t PIDbitrate;
    if (!firstPCRundefined && !(packet->isDiscontunityIndicator()) ){
        const uint64_t delta = packet->getPacketNo() - firstPCRPacketNo;
        if (newPCR.calculatePCR() - firstPCR.calculatePCR()>0){
            PIDbitrate = (delta * MPEG_PACKET_SIZE * SYS_CLOCK_F * 8)/(newPCR.calculatePCR() - firstPCR.calculatePCR());
            bitrates[packet->getPID()]=PIDbitrate;
            uint64_t brbuffer = 0;
            for (auto const& localbitrate : bitrates)
                brbuffer+=localbitrate.second;
            streambitrate=brbuffer/bitrates.size();
        }
    }
    else{
        firstPCRPackets[packet->getPID()]=packet;
    }
}

class Multiplex {
    uint16_t PID;
    string sPname,sName, bitrate;
    
public:
    Multiplex(uint16_t pid, string sp, string sn, double br):
    PID(pid), sPname(sp), sName(sn){
        br/=1000000;
        stringstream s;
        s<<fixed<<setprecision(2)<<setfill('0')<<br;
        bitrate=s.str()+" Mbps";
    }
    
    bool operator <  (Multiplex& m) { return (PID <  m.PID); }
    bool operator >  (Multiplex& m) { return (PID >  m.PID); }
    bool operator == (Multiplex& m) { return (PID == m.PID); }
    bool operator <= (Multiplex& m) { return (PID <= m.PID); }
    bool operator >= (Multiplex& m) { return (PID >= m.PID); }
    
    uint16_t getPID(){ return PID; }
    string getServiceProvider(){ return sPname; }
    string getService() { return sName; }
    string getBitrate() { return bitrate; }    
    
    string getPIDString(){
        stringstream s;
        s<<"0x"<< setfill('0') << setw(4) << hex << getPID();
        return s.str();
    }
};

class printer {
    Tables<NIT> NITs;
    Tables<SDT> SDTs;
    Tables<PAT> PATs;
    vector<MPEGPacket*> packets, unmergedpackets;
    
    public:
    printer(Tables<NIT>& nits,Tables<SDT>& sdts,Tables<PAT>& pats,vector<MPEGPacket*>& pks,vector<MPEGPacket*>& umpks):
    NITs(nits),SDTs(sdts),PATs(pats),packets(pks),unmergedpackets(umpks){}
    
    string getOutput(){
        string result;
        NIT nitdata = NITs[0];
        
        result+="Network name: "+nitdata.getNetworkname()+"\n";
        result+="Network ID: "+to_string(nitdata.getNetworkId())+"\n";
        
        vector<TS_NITInfo*> tsinfos = nitdata.getTSInfos(); //assume we have 1 TS
        if (tsinfos.size()==0){
            cerr<<"Error: No TS info found in NIT\n";
            exit(1);
        }         
        
        result+="Bandwidth: "+tsinfos[0]->getBandwidth()+"\n";
        result+="Constellation: "+tsinfos[0]->getConstellation()+"\n";
        result+="Guard interval: "+tsinfos[0]->getGuardInterval()+"\n";
        result+="Code rate: "+tsinfos[0]->getCodeRate()+"\n";
        result+="\n";
        
        vector<Multiplex*> multiplexes = getMultiplexes();
        for (auto multiplex: multiplexes){            
            result += multiplex->getPIDString() +"-" +multiplex->getServiceProvider()+"-"+
                      multiplex->getService()   +": "+multiplex->getBitrate()       +"\n";
        }
        
        return result;
    }
    vector<Multiplex*> getMultiplexes(){
        vector<Multiplex*> v;        
        SDT sdtdata = SDTs[0];
        PAT patdata = PATs[0];
        
        vector<Service*> services = sdtdata.getServices();
        for (Service* service: services){
            string sname = service->getServiceName();
            string sprovider = service->getServiceProviderName();
            uint16_t pid = 0;
            double bitrate = 0;
            
            //The service_id is the same as the program_number
            //in the corresponding program_map_section.
            PMT* pmtdataptr = nullptr;
            for (Program* program: patdata.getPrograms()){
                if (program->getProgramNumber() == service->getServiceID()){
                    pid = program->getProgramMapPID();
                    pmtdataptr = patdata.getPMT(program->getProgramNumber(),packets);
                    break;
                }
            }
            if (!pmtdataptr){
                cerr<<"Error: PMT for program number "<<service->getServiceID()<<" was not found\n";
                exit(1);                
            }
            vector<uint16_t> multiplexPIDs = pmtdataptr->getComponendPIDs();
            bitrate = calculateLocalBitrate(multiplexPIDs);
            
            v.push_back(new Multiplex(pid,sprovider,sname,bitrate));
        }
        sort(v.begin(),v.end(),
            [] (Multiplex* a, Multiplex* b) -> bool { return (a && b)?(*a)<(*b) :a<b; });
        return v;
    }
    double calculateLocalBitrate(vector<uint16_t>& multiplexPIDs){
        double total_packets = unmergedpackets.size();
        unsigned program_packets = 0;
        //Lets count all packets with PID's corresponding to certain program
        for (auto mPID : multiplexPIDs){
            for (auto packet: unmergedpackets){
                if (packet->getPID() == mPID) program_packets++;
            }
        }
        //According to BMS2 project task, calculate local bitrate
        return (((double)program_packets)/((double)total_packets))*((double)streambitrate);
    }
};


int main(int argc, char* argv[])
{
	if (argc != 2) {
		cerr << "Program expects 1 argument\n";
		return 1;
	}
	ifstream input(argv[1], std::ios::binary | ios::in);
	if (!input.good()) {
		cerr << "Could not open file " << argv[1] << "\n";
		return 1;
	}
    string rawname(argv[1]);
    size_t lastindex = rawname.find_last_of("."); 
    rawname = rawname.substr(0, lastindex);
    
	input.unsetf(std::ios::skipws);


	//Define collections for relevant data
	Tables<NIT> NITs;
    Tables<SDT> SDTs;
    Tables<PAT> PATs;
    vector<MPEGPacket*> packets;    
    
    vector<uint8_t> rawpacket(MPEG_PACKET_SIZE,0);  
	
    //read all packets    
    while (readpacket(rawpacket, input)){
        MPEGPacket* packet = new MPEGPacket(rawpacket);
        packets.push_back(packet);
        updateBitrate(packet);
    }    
	input.close();
    vector<MPEGPacket*> unmergedpackets = packets;
    //merge split sections, keep only packets we need
    mergePackets(packets);
    
    for (auto packet: packets){
        switch (packet->getPID()) {
            case NIT_PID:
                NITs.add(new NIT(packet));
                break;
            case PAT_PID:
                PATs.add(new PAT(packet));
                break;
            case SDT_PID:
                SDTs.add(new SDT(packet));
                break;            
            default:
                break;
        }
    }
    if (!NITs.size()){
        cerr<<"Error, no NIT table was found\n";
        return 1;
    }
    if (!SDTs.size()){
        cerr<<"Error, no SDT table was found\n";
        return 1;
    }
    if (!PATs.size()){
        cerr<<"Error, no PAT table was found\n";
        return 1;
    }
    
    printer Printer(NITs,SDTs,PATs,packets,unmergedpackets);    
    string outputbuffer = Printer.getOutput();
    #if DEBUG
        cout<<"Stream bitrate: "<<streambitrate<<"\n";
        cout<<outputbuffer;
    #endif
    string ofilename= rawname+".txt";
    ofstream output(ofilename.c_str(), std::ios::binary | ios::out);
	if (!output.good()) {
		cerr << "Could not create output file " << ofilename.c_str() << "\n";
		return 1;
	}
    output<<outputbuffer;
    output.close();
    
    NITs.cleanup();
    PATs.cleanup();
    SDTs.cleanup();
	return 0;
}