
#ifndef CSVREADER_H
#define CSVREADER_H

#include <iostream>     // cout, endl
#include <fstream>      // fstream
#include <vector>
#include <string>
#include <boost/tokenizer.hpp>


///	\class CSVReader
///	\author Luigi
///	\brief A class for reading csv files 
///	\note 
/// 	\todo 
///	\date
///	\warning
class CSVReader
{
    typedef boost::tokenizer< boost::escaped_list_separator<char> > Tokenizer;
    
public: 
    
    // basic constructor: open file and parse it 
    CSVReader(const std::string& filename)
    {
        str_filename_ = filename;

        std::ifstream in(str_filename_);
        if (!in.is_open()) 
        {
            std::cout << "CSVReader::parse() - error opening file: " << str_filename_ << std::endl; 
        }
                
        std::vector< std::string > vec;
        std::string line;
    
        int i = 0; 
        while (std::getline(in,line))
        {
            Tokenizer tok(line);
            vec.assign(tok.begin(),tok.end());

            mat_data_.push_back(std::vector<double>());
            for(int j=0; j<vec.size(); j++)
            {
                mat_data_[i].push_back(std::stod(vec[j])); 
            }
            i++;
        }
    }
    
public: // getters 
    
    // get the mat in which the data have been put 
    std::vector< std::vector<double> >& getData() { return mat_data_; }
    
public: // utils 
    
    void print(std::ostream& str = std::cout)
    {
        for(int i=0; i<mat_data_.size(); i++)
        {
            for(int j=0; j<mat_data_[i].size(); j++)
            {
                str << mat_data_[i][j] << " ";
            }
            str << std::endl; 
        } 
    }
    
  
public: 
    
    std::string str_filename_; 
    std::vector< std::vector<double> > mat_data_; 
    
};


inline std::ostream& operator<<(std::ostream& str, CSVReader& csv_reader)
{
    csv_reader.print(str); 
    return str;
}


/// < for testing 
//int main(int argc, char* argv[])
//{
//    std::string csv_filename;
//    if(argc==1)
//    {
//        std::cout << "enter input filename" << std::endl; 
//    }
//    else
//    {
//        csv_filename = argv[1];
//        std::cout << "opening file: " << csv_filename << std::endl;
//    }
//    
//    CSVReader csv_reader(csv_filename); 
//    std::vector< std::vector<double> >& mat_data =  csv_reader.getData();
//    csv_reader.print(); 
//}


#endif /* CSVREADER_H */

