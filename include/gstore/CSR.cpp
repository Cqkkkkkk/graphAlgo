#include "CSR.h"

using namespace std;

CSR::CSR()
{

}

CSR::CSR(unsigned pnum)
{
	this->w = 4;
	this->pre_num = pnum;
	this->id2vid = new vector<unsigned>[this->pre_num];
	this->vid2id = new map<unsigned, unsigned>[this->pre_num];
	this->offset_list = new vector<unsigned>[this->pre_num];
	this->adjacency_list = new vector<unsigned>[this->pre_num];
	// this->valid = new bool[this->pre_num];
}

CSR::~CSR()
{
	
}

void CSR::init(unsigned pnum)
{
	this->w = 4;
	this->pre_num = pnum;
	this->id2vid = new vector<unsigned>[this->pre_num];
	this->vid2id = new map<unsigned, unsigned>[this->pre_num];
	this->offset_list = new vector<unsigned>[this->pre_num];
	this->adjacency_list = new vector<unsigned>[this->pre_num];
	this->n = 0;
	this->m = 0;
}

void CSR::print()
{
	unsigned i, j;
	for (i = 0; i < pre_num; i++)
	{
		cout << "Predicate ID = " << i << endl;

		cout << "	id2vid:" << endl;
		cout << "	";
		for (j = 0; j < id2vid[i].size(); j++)
			cout << id2vid[i][j] << ' ';
		cout << endl;

		cout << "	vid2id:" << endl;
		cout << "	";
		for (j = 0; j < vid2id[i].size(); j++)
			cout << vid2id[i][j] << ' ';
		cout << endl;

		cout << "	Offset list:" << endl;
		cout << "	";
		for (j = 0; j < offset_list[i].size(); j++)
			cout << offset_list[i][j] << ' ';
		cout << endl;

		cout << "	Adjacency list:" << endl;
		cout << "	";
		for (j = 0; j < adjacency_list[i].size(); j++)
			cout << adjacency_list[i][j] << ' ';
		cout << endl;
	}
}

long long CSR::sizeInBytes()
{
	long long sz = 0;
	for (unsigned int i = 0; i < pre_num; i++)
		sz += 2 * id2vid[i].size() + offset_list[i].size() + adjacency_list[i].size();
	sz *= 8;
	return sz;
}