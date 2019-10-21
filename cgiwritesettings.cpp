/*----------------------------------------------------------------------
 * Copyright (c) 2019 XIA LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, 
 * with or without modification, are permitted provided 
 * that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above 
 *     copyright notice, this list of conditions and the 
 *     following disclaimer.
 *   * Redistributions in binary form must reproduce the 
 *     above copyright notice, this list of conditions and the 
 *     following disclaimer in the documentation and/or other 
 *     materials provided with the distribution.
 *   * Neither the name of XIA LLC
 *     nor the names of its contributors may be used to endorse 
 *     or promote products derived from this software without 
 *     specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
 * SUCH DAMAGE.
 *----------------------------------------------------------------------*/
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <sys/mman.h>

#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <algorithm>

#include "PixieNetDefs.h"
#include "PixieNetCommon.h"

using namespace std;

  // trim from start
  static inline std::string &ltrim(std::string &s)
  {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    
    if( s.size() )
    {
      const size_t pos = s.find_first_not_of( '\0' );
      if( pos != 0 && pos != string::npos )
        s.erase( s.begin(), s.begin() + pos );
      else if( pos == string::npos )
        s.clear();
    }
    
    return s;
  }
  
  // trim from end
  static inline std::string &rtrim(std::string &s)
  {
    s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    
    const size_t pos = s.find_last_not_of( '\0' );
    if( pos != string::npos && (pos+1) < s.size() )
      s.erase( s.begin() + pos + 1, s.end() );
    else if( pos == string::npos )
      s.clear();  //string is all '\0' characters
    
    return s;
  }

  // trim from both ends
  void trim( std::string &s )
  {
    ltrim( rtrim(s) );
  }//trim(...)


  std::istream &safe_get_line( std::istream &is, std::string &t, const size_t maxlength )
  {
    //adapted from  http://stackoverflow.com/questions/6089231/getting-std-ifstream-to-handle-lf-cr-and-crlf
    t.clear();
    
    // The characters in the stream are read one-by-one using a std::streambuf.
    // That is faster than reading them one-by-one using the std::istream.
    // Code that uses streambuf this way must be guarded by a sentry object.
    // The sentry object performs various tasks,
    // such as thread synchronization and updating the stream state.
    std::istream::sentry se( is, true );
    std::streambuf *sb = is.rdbuf();
    
    for( ; !maxlength || (t.length() < maxlength); )
    {
      int c = sb->sbumpc(); //advances pointer to current location by one
      switch( c )
      {
        case '\r':
          c = sb->sgetc();  //does not advance pointer to current location
          if(c == '\n')
            sb->sbumpc();   //advances pointer to one current location by one
          return is;
        case '\n':
          return is;
        case EOF:
          is.setstate( ios::eofbit );
          return is;
        default:
          t += (char)c;
      }//switch( c )
    }//for(;;)
    
    return is;
  }//safe_get_line(...)

    bool split_label_values( const string &line, string &label, string &values )
  {
    label.clear();
    values.clear();
    const size_t pos = line.find_first_of( " \t,;&=" );
    if( !pos || pos == string::npos )
      return false;
    label = line.substr( 0, pos );
    values = line.substr( pos );
    trim( values );
    
    return true;
  }


int main(void) {

//  int k;
//  FILE * fil;
//  char line[LINESZ];
   
 


  // ************** XIA code begins **************************

    string line;
    string newline;
    string label, values;
    string strReplace = "MODULE_ID";
    string strNew = "MODULE_ID       7758";
    string remainder;
 //   int total_n=0;
//    int n;
    char * linecomp ="<none>";

  char * data;
   float val[32] = {0.0};
  data = getenv("QUERY_STRING");
   
   std::string webdata(data);
   split_label_values( webdata, strReplace, remainder );      // extract label from webdata (break on &)
  // split_label_values( strReplace, label, strReplace );      // extract label from webdata (break on =, use 2nd half)

  /*
  k=0;
  while (1 == sscanf(data + total_n, "%*[^0123456789]%d%n", &val[k], &n))
   {
       total_n += n;
       printf("%d\n", val[k]);
       k++;
   }

   sscanf(data,"%d %d %d %d %d %d %d %d",&v00, &v01, &v02, &v03, &v04, &v05, &v06, &v07);
   sprintf(linecomp,"%s    %d  %d   %d  %d   %d  %d   %d  %d",strReplace.c_str(),val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7]);  

 //  std::string strNew(linecomp);
*/

   stringstream ss(remainder);
   ss >> val[0];
   ss >> val[1];

  // sprintf(linecomp," %4.3f %4.3f",val[0], val[1]);
 //  sprintf(linecomp," helloe %f",0.1234);

   strNew = remainder;
 
  // read the settings file and replace a line




   ifstream filein( "settings.ini");
   ofstream fileout("settings.txt"); 
   if(!filein || !fileout)
    {
        cout << "Error opening files on server!" << endl;
        return -1;
    }



    while( safe_get_line( filein, line, LINESZ ) )  // get lines
    {
    
      split_label_values( line, label, values );      // extract label from line

       int equal;
       equal = strReplace.compare(label);
       
       if(equal!=0)
               newline = line;
       else
          //     newline = line;
               newline = strNew;
   
       newline += "\n";
       fileout << newline;
    }

  //  strReplace += "\n";
    fileout << strReplace;

//    std::string strNew2(linecomp);
//    fileout <<strNew2; 
 //   fileout << linecomp;

//    printf("Settings file updated on server, parameter %s\n",strReplace.c_str());

  //  remove("settings.ini");
  //  rename("settings.txt","settings.ini");
   
 // clean up  
 //fclose(fil);
 return 0;
}






/*
int replace_config_file_lines( const char * const filename,
                               const char * const linereplacement,
                               const char * const targetlabel)
{
 
string line;
string label, values;
std::string targetstr(targetlabel);

 cerr << "linereplacement" << linereplacement << endl;
 cerr << "targetlabel" << targetlabel << endl;
 
 ifstream input( filename, ios::in | ios::binary );
 
 if( !input )
 {
   cerr << "Failed to open '" << filename << "'" << endl;
   return -1;
 }

 ofstream output("settings.tmp"); 
 
 while( safe_get_line( input, line, LINESZ ) )                          // get lines
 {
 //  trim( line );
 //  if( line.empty() || line[0] == '#' )                                 // check for empty lines
 //    continue;
   

   const bool success = split_label_values( line, label, values );      // extract label from line
   
   if( !success || label.empty() || values.empty() )
   {
     cerr << "Warning: encountered invalid config file line '" << line
     << "', skipping" << endl;
     continue;
   }
   
   int equal;
   equal = label.compare(targetstr);
   if(equal!=0)
      printf("%s\n",line.c_str());
   else
      printf("%s\n",linereplacement);


 }///more lines in config file
 
 return 0;
}//replace_config_file_lines(..)

*/
