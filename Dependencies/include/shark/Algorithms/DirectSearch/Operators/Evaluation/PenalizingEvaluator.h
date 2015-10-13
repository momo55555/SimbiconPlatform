//===========================================================================
/*!
 * 
 *
 * \brief       PenalizingEvaluator


 * 
 *
 * \author      T. Voss, O.Krause
 * \date        2014
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

#ifndef SHARK_ALGORITHMS_DIRECT_SEARCH_OPERATORS_EVALUATION_PENALIZING_EVALUATOR_H
#define SHARK_ALGORITHMS_DIRECT_SEARCH_OPERATORS_EVALUATION_PENALIZING_EVALUATOR_H

#include <shark/LinAlg/Base.h>

namespace shark {
/**
* \brief Penalizing evaluator for scalar objective functions.
*
* Evaluates the supplied single-objective function \f$f\f$ for the search point \f$s\f$
* according to:
* \f{align*}{
*   y & = & f( s' )\\
*   y' & = & f( s' ) + \alpha \vert\vert s - s' \vert\vert_2^2
* \f}
* where \f$s'\f$ is the repaired version of \f$s\f$ if \f$s\f$ is not feasible and equal to \f$s\f$ otherwise.
* The default value of \f$\alpha\f$ is \f$10^{-6}\f$.
*/
struct PenalizingEvaluator {
	/**
	* \brief Default c'tor, initializes the penalty factor to \f$10^{-6}\f$.
	*/
	PenalizingEvaluator() : m_penaltyFactor( 1E-6 ) {}

	/**
	* \brief Evaluates the supplied function on the supplied individual
	*
	* \param [in] f The function to be evaluated.
	* \param [in] individual The individual to evaluate the function for.
	*/
	template<typename Function, typename IndividualType>
	void operator()( Function const& f, IndividualType& individual ) const {

		if( f.isFeasible( individual.searchPoint() ) ) {
			individual.unpenalizedFitness() = f.eval( individual.searchPoint() );
			individual.penalizedFitness() = individual.unpenalizedFitness();
			return;
		}

		typename Function::SearchPointType t( individual.searchPoint() );
		f.closestFeasible( t );

		individual.unpenalizedFitness() = f.eval( t );
		individual.penalizedFitness() = individual.unpenalizedFitness();
		
		penalize(individual.searchPoint(),t,individual.penalizedFitness() );
	}
	
	/**
	* \brief Evaluates The function on individuals in the range [first,last]
	*
	* \param [in] f The function to be evaluated.
	* \param [in] begin first indivdual in the range to be evaluated
	* \param [in] end iterator pointing directly beehind the last individual to be evaluated
	*/
	template<typename Function, typename Iterator>
	void operator()( Function const& f, Iterator begin, Iterator end ) const {
		for(Iterator pos = begin; pos != end; ++pos){
			(*this)(f,*pos);
		}
	}
	
	template<class SearchPointType>
	void penalize(SearchPointType const& s, SearchPointType const& t, double& fitness)const{
		fitness += m_penaltyFactor * norm_sqr( t - s );
	}
	
	template<class SearchPointType>
	void penalize(SearchPointType const& s, SearchPointType const& t, RealVector& fitness)const{
		fitness += m_penaltyFactor * norm_sqr( t - s ) * blas::repeat(1.0,fitness.size());
	}

	/**
	* \brief Stores/loads the evaluator's state.
	* \tparam Archive The type of the archive.
	* \param [in,out] archive The archive to use for loading/storing.
	* \param [in] version Currently unused.
	*/
	template<typename Archive>
	void serialize( Archive & archive, const unsigned int version ) {
		archive & m_penaltyFactor;
	}

	double m_penaltyFactor; ///< Penalty factor \f$\alpha\f$, default value: \f$10^{-6}\f$ .

};
}


#endif
