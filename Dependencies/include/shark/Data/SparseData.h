//===========================================================================
/*!
 * 
 *
 * \brief   Support for importing and exporting data from and to sparse data (libSVM) formatted data files
 * 
 * 
 * \par
 * The most important application of the methods provided in this
 * file is the import of data from LIBSVM files to Shark Data containers.
 * 
 * 
 * 
 *
 * \author      M. Tuma, T. Glasmachers, C. Igel
 * \date        2010
 *
 *
 * \par Copyright 1995-2015 Shark Development Team
 * 
 * <BR><HR>
 * This file is part of Shark.
 * <http://image.diku.dk/shark/>
 * 
 * Shark is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published 
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * Shark is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with Shark.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
//===========================================================================

#ifndef SHARK_DATA_SPARSEDATA_H
#define SHARK_DATA_SPARSEDATA_H

#include <shark/Core/DLLSupport.h>
#include <fstream>
#include <shark/Data/Dataset.h>

namespace shark {

namespace detail {

typedef std::pair< unsigned int, size_t > LabelSortPair;
static inline bool cmpLabelSortPair(const  LabelSortPair& left, const LabelSortPair& right) {
	return left.first > right.first; // for sorting in decreasing order
}

} // namespace detail

/**
 * \ingroup shark_globals
 *
 * @{
 */



/// \brief Import data from a sparse data (libSVM) file.
///
/// \param  dataset       container storing the loaded data
/// \param  stream        stream to be read from
/// \param  highestIndex  highest feature index, or 0 for auto-detection
/// \param  batchSize     size of batch
SHARK_EXPORT_SYMBOL void importSparseData(
	LabeledData<RealVector, unsigned int>& dataset,
	std::istream& stream,
	unsigned int highestIndex = 0,
	std::size_t batchSize = LabeledData<RealVector, unsigned int>::DefaultBatchSize
);

/// \brief Import data from a sparse data (libSVM) file.
///
/// \param  dataset       container storing the loaded data
/// \param  stream        stream to be read from
/// \param  highestIndex  highest feature index, or 0 for auto-detection
/// \param  batchSize     size of batch
SHARK_EXPORT_SYMBOL void importSparseData(
	LabeledData<CompressedRealVector, unsigned int>& dataset,
	std::istream& stream,
	unsigned int highestIndex = 0,
	std::size_t batchSize = LabeledData<RealVector, unsigned int>::DefaultBatchSize
);

/// \brief Import data from a sparse data (libSVM) file.
///
/// \param  dataset       container storing the loaded data
/// \param  fn            the file to be read from
/// \param  highestIndex  highest feature index, or 0 for auto-detection
/// \param  batchSize     size of batch
SHARK_EXPORT_SYMBOL void importSparseData(
	LabeledData<RealVector, unsigned int>& dataset,
	std::string fn,
	unsigned int highestIndex = 0,
	std::size_t batchSize = LabeledData<RealVector, unsigned int>::DefaultBatchSize
);

/// \brief Import data from a sparse data (libSVM) file.
///
/// \param  dataset       container storing the loaded data
/// \param  fn            the file to be read from
/// \param  highestIndex  highest feature index, or 0 for auto-detection
/// \param  batchSize     size of batch
SHARK_EXPORT_SYMBOL void importSparseData(
	LabeledData<CompressedRealVector, unsigned int>& dataset,
	std::string fn,
	unsigned int highestIndex = 0,
	std::size_t batchSize = LabeledData<RealVector, unsigned int>::DefaultBatchSize
);


/// \brief Export data to sparse data (libSVM) format.
///
/// \param  dataset     Container storing the  data
/// \param  fn          Output file
/// \param  dense       Flag for using dense output format
/// \param  oneMinusOne Flag for applying the transformation y<-2y-1 to binary labels
/// \param  sortLabels  Flag for sorting data points according to labels
/// \param  append      Flag for appending to the output file instead of overwriting it
template<typename InputType>
void exportSparseData(LabeledData<InputType, unsigned int>& dataset, const std::string &fn, bool dense=false, bool oneMinusOne = true, bool sortLabels = false, bool append = false) {
	std::size_t elements = dataset.numberOfElements();
    std::ofstream ofs;
    
    // shall we append only or overwrite?
    if (append == true) {
        ofs.open (fn.c_str(), std::fstream::out | std::fstream::app );
    } else {
        ofs.open (fn.c_str());
    }
    
	if( !ofs ) {
		throw( SHARKEXCEPTION( "[exportSparseData] file can not be opened for writing" ) );
	}

	size_t dim = inputDimension(dataset);
	if(numberOfClasses(dataset)!=2) oneMinusOne = false;

	std::vector<detail::LabelSortPair> L;
	if(sortLabels) {
		for(std::size_t i = 0; i < elements; i++) 
			L.push_back(detail::LabelSortPair(dataset.element(i).label, i));
		std::sort (L.begin(), L.end(), detail::cmpLabelSortPair);
	}

	for(std::size_t ii = 0; ii < elements; ii++) {
		// apply mapping to sorted indices
		std::size_t i = 0;
		if(sortLabels) i = L[ii].second;
		else i = ii;
		// apply transformation to label and write it to file
		if(oneMinusOne) ofs << 2*int(dataset.element(i).label)-1 << " ";
		//libsvm file format documentation is scarce, but by convention the first class seems to be 1..
		else ofs << dataset.element(i).label+1 << " ";
		// write input data to file
		for(std::size_t j=0; j<dim; j++) {
			if(dense) 
				ofs << " " << j+1 << ":" <<dataset.element(i).input(j);
			else if(dataset.element(i).input(j) != 0) 
				ofs << " " << j+1 << ":" << dataset.element(i).input(j);
		}
		ofs << std::endl;
	}
}

/** @}*/

}
#endif
