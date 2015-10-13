/*!
 * 
 *
 * \brief       error function for supervised learning
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
#ifndef SHARK_OBJECTIVEFUNCTIONS_ERRORFUNCTION_H
#define SHARK_OBJECTIVEFUNCTIONS_ERRORFUNCTION_H


#include <shark/Models/AbstractModel.h>
#include <shark/ObjectiveFunctions/Loss/AbstractLoss.h>
#include <shark/ObjectiveFunctions/AbstractObjectiveFunction.h>
#include <shark/Data/Dataset.h>
#include "Impl/FunctionWrapperBase.h"

#include <boost/scoped_ptr.hpp>

namespace shark{

///
/// \brief Objective function for supervised learning
///
/// \par
/// An ErrorFunction object is an objective function for
/// learning the parameters of a model from data by means
/// of minimization of a cost function. The value of the
/// objective function is the cost of the model predictions
/// on the training data, given the targets.
///
/// \par
/// The class detects automatically when an AbstractLoss is used 
/// as Costfunction. In this case, it uses faster algorithms 
/// for empirical risk minimization
///
///\par
/// It also automatically infers the input und label type from the given dataset and the output type
/// of the model in the constructor and ensures that Model and loss match. Thus the user does
/// not need to provide the types as template parameters. 
class ErrorFunction : public SingleObjectiveFunction
{
public:
	template<class InputType, class LabelType, class OutputType>
	ErrorFunction(
		LabeledData<InputType, LabelType> const& dataset,
		AbstractModel<InputType,OutputType>* model, 
		AbstractLoss<LabelType, OutputType>* loss
		
	);
	ErrorFunction(const ErrorFunction& op);
	ErrorFunction& operator=(const ErrorFunction& op);

	std::string name() const
	{ return "ErrorFunction"; }
	
	void setRegularizer(double factor, SingleObjectiveFunction* regularizer){
		m_regularizer = regularizer;
		m_regularizationStrength = factor;
	}

	void proposeStartingPoint(SearchPointType& startingPoint) const;
	std::size_t numberOfVariables()const;

	double eval(RealVector const& input) const;
	ResultType evalDerivative( const SearchPointType & input, FirstOrderDerivative & derivative ) const;
	
	friend void swap(ErrorFunction& op1, ErrorFunction& op2);

private:
	boost::scoped_ptr<detail::FunctionWrapperBase > mp_wrapper;
	SingleObjectiveFunction* m_regularizer;
	double m_regularizationStrength;
};

}
#include "Impl/ErrorFunction.inl"
#endif
