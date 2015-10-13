/*!
 * 
 *
 * \brief       AbstractMultiObjectiveOptimizer
 * 
 * 
 *
 * \author      T.Voss, T. Glasmachers, O.Krause
 * \date        2010-2011
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
#ifndef SHARK_OBJECTIVEFUNCTIONS_ABSTRACTMULTIOBJECTIVEOPTIMIZER_H
#define SHARK_OBJECTIVEFUNCTIONS_ABSTRACTMULTIOBJECTIVEOPTIMIZER_H

#include <shark/Algorithms/AbstractOptimizer.h>
#include <shark/Core/ResultSets.h>

namespace shark {

/**
 * \brief base class for abstract multi-objective optimizers for arbitrary search spaces.
 *
 * Models an abstract multi-objective optimizer for arbitrary search spaces. The objective space
 * is assumed to be \f$ \mathbb{R}^m\f$.
 *
 * \tparam PointType The type of the points that make up the searchspace.
 */
template<typename PointTypeT>
class AbstractMultiObjectiveOptimizer : 
public AbstractOptimizer<
	PointTypeT,
	RealVector,
	std::vector< ResultSet< PointTypeT, RealVector > > 
> {
private:
typedef AbstractOptimizer<
	PointTypeT,
	RealVector,
	std::vector< ResultSet< PointTypeT, RealVector > > 
> super;
public:
	typedef typename super::SearchPointType SearchPointType;
	typedef typename super::SolutionType SolutionType;
	typedef typename super::ObjectiveFunctionType ObjectiveFunctionType;

	/**
	* \brief Virtual empty d'tor.
	*/
	virtual ~AbstractMultiObjectiveOptimizer() {}

	/**
	* \brief Initializes the optimizer for the supplied objective function.
	*
	* Tries to sample an initial starting point. If the function does not
	* implement this feature, an exception is thrown. Otherwise, the call is dispatched
	* to the pure-virtual function.
	*
	* \param function The function to be initialized for.
	* \throws Exception if the function does not feature the proposal of starting points.
	*/
	virtual void init( const ObjectiveFunctionType & function ) {
		if(!(function.features() & ObjectiveFunctionType::CAN_PROPOSE_STARTING_POINT))
			throw SHARKEXCEPTION( "Objective function does not propose a starting point");
		std::vector<RealVector> startingPoints(1);
		function.proposeStartingPoint(startingPoints[0]);
		init(function, startingPoints);
	}

	/**
	* \brief Optimizer-specific init-function. Needs to be implemented by subclasses.
	* \param [in] function The function to initialize the optimizer for.
	* \param [in] startingPoints An initial population of points
	*/
	virtual void init( 
		ObjectiveFunctionType const& function, 
		std::vector<SearchPointType> const& startingPoints
	) = 0;

	/**
	* \brief Accesses the current approximation of the Pareto-set and -front, respectively.
	* \returns The current set of candidate solutions.
	*/
	const SolutionType & solution() const {
		return m_best;
	}

protected:
	SolutionType m_best; ///< The current Pareto-set/-front.
};

}
#endif