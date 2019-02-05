/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Garratt Gallagher
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name Garratt Gallagher nor the names of other
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/



#ifndef PRINTING_TOOLS_H_
#define PRINTING_TOOLS_H_

void printAll( string s1, string s2, vector<int> &data){
    int maxsize=0;
    for(uint i=0;i<data.size();i++)
    	if(data[i]>maxsize) maxsize=data[i];
    std::vector<int> ocounts(maxsize+1,0);
    for(uint i=0;i<data.size();i++)
    	ocounts[data[i]]++;
    for(uint i=0;i<ocounts.size();i++)
    	if(ocounts[i])
    		std::cout<<setw(6)<<ocounts[i]<<" "<<s1<<i<<" "<<s2<<std::endl;
}


void miniHist(int indmax, vector<int> bounds, string s1, string s2, vector<int> &data){
	std::vector<int> counts(indmax+bounds.size()+2,0);
	bounds.push_back(INT_MAX); //this makes it easier for assigning...
    for(uint i=0;i<data.size();i++)
    	if(data[i]<=indmax)
    		counts[data[i]]++;
    	else{
    		for(uint j=0;j<bounds.size();j++) //data[i] is guaranteed to be w/in bounds
    			if(data[i]<=bounds[j]){
    				counts[indmax+j+1]++;
    				break;
    			}
    	}
    //now print out the summary:
    //for the individual counts:
    for(uint i=0;i<=indmax;i++)
    	if(counts[i])
    		std::cout<<setw(6)<<counts[i]<<" "<<s1<<setw(4)<<i<<" "<<s2<<std::endl;
    //between indmax and the first bounds
	if(counts[indmax+1])
		std::cout<<setw(6)<<counts[indmax+1]<<" "<<s1<<setw(4)<<" between "<<setw(4)<<indmax+1<<" and "<<setw(4)<<bounds[0]<<" "<<s2<<std::endl;
	//between the given bounds
	for(uint i=0;i<bounds.size()-2;i++)
		if(counts[indmax+2+i])
			std::cout<<setw(6)<<counts[indmax+2+i]<<" "<<s1<<setw(4)<<" between "<<setw(4)<<bounds[i]+1<<" and "<<setw(4)<<bounds[i+1]<<" "<<s2<<std::endl;
	//greater than the last bound
	bounds.pop_back();
	if(counts.back())
		std::cout<<setw(6)<<counts.back()<<" "<<s1<<setw(4)<<" more than "<<setw(4)<<bounds.back()+1<<" "<<s2<<std::endl;

}




#endif /* PRINTING_TOOLS_H_ */
