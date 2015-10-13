/*!
 * 
 *
 * \brief       Convex quadratic benchmark function.
 * 
 *
 * \author      T.Voss
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
#ifndef SHARK_OBJECTIVEFUNCTIONS_BENCHMARKS_ELLIPSOID_H
#define SHARK_OBJECTIVEFUNCTIONS_BENCHMARKS_ELLIPSOID_H

#include <shark/ObjectiveFunctions/AbstractObjectiveFunction.h>
#include <shark/Rng/GlobalRng.h>

namespace shark {
/**
*  \brief Convex quadratic benchmark function
*
*  The eigenvalues of the Hessian of this convex quadratic benchmark
*  function are equally distributed on logarithmic scale.
*/
struct Ellipsoid : public SingleObjectiveFunction {
	Ellipsoid(size_t numberOfVariables = 5, double alpha=1E-3) : m_alpha(alpha) {
		m_features |= CAN_PROPOSE_STARTING_POINT;
		m_features |= HAS_FIRST_DERIVATIVE;
		m_features |= HAS_SECOND_DERIVATIVE;
		m_numberOfVariables = numberOfVariables;
	}

	/// \brief From INameable: return the class name.
	std::string name() const
	{ return "Ellipsoid"; }
	
	std::size_t numberOfVariables()const{
		return m_numberOfVariables;
	}
	
	bool hasScalableDimensionality()const{
		return true;
	}
	
	void setNumberOfVariables( std::size_t numberOfVariables ){
		m_numberOfVariables = numberOfVariables;
	}

	void proposeStartingPoint( SearchPointType & x ) const {
		x.resize( m_numberOfVariables );

		for( unsigned int i = 0; i < x.size(); i++ ) {
			x( i ) = Rng::uni( 0, 1 );
		}
	}

	double eval( const SearchPointType & p ) const {
		m_evaluationCounter++;
		double sum = 0;
		double sizeMinusOne = p.size() - 1.;
		for( unsigned int i = 0; i < p.size(); i++ ){
			sum += ::pow( m_alpha, i / sizeMinusOne ) * sqr(p( i ) );
		}

		return sum;
	}

	double evalDerivative( const SearchPointType & p, FirstOrderDerivative & derivative ) const {
		double sizeMinusOne=p.size() - 1.;
		derivative.resize(p.size());
		for (unsigned int i = 0; i < p.size(); i++) {
			derivative(i) = 2 * ::pow(m_alpha, i / sizeMinusOne) * p(i);
		}
		return eval(p);
	}
	double evalDerivative(const SearchPointType &p, SecondOrderDerivative &derivative)const {
		std::size_t size=p.size();
		double sizeMinusOne=p.size() - 1.;
		derivative.m_gradient.resize(size);
		derivative.m_hessian.resize(size,size);
		derivative.m_hessian.clear();
		for (unsigned int i = 0; i < size; i++) {
			derivative.m_gradient(i) = 2 * std::pow(m_alpha, i / sizeMinusOne ) * p(i);
			derivative.m_hessian(i,i) = 2 * std::pow(m_alpha, i /sizeMinusOne );
		}
		return eval(p);
	}
private:
	std::size_t m_numberOfVariables;
	double m_alpha;
};

}

#endif
