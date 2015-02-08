/*------------------------------------------------------------------------------
 *                  __                           ___
 *                 || |             __          //  |
 *       _______   || |   _______  || |__      //   |    _____  ___
 *      ||  ___ \  || |  // ___  | ||  __|    // _  |   ||  _ \/ _ \
 *      || |  || | || | || |  || | || |      // /|| |   || |\\  /\\ \
 *      || |__|| | || | || |__|| | || |     // /_|| |_  || | || | || |
 *      ||  ____/  || |  \\____  | || |__  //_____   _| || | || | || |
 *      || |       ||_|       ||_|  \\___|       ||_|   ||_| ||_| ||_|
 *      || |
 *      ||_|
 *
 * Copyright (c) 2013 Ben Minerd. All rights reserved.
 *
 * GNU Lesser General Public License Usage
 * This file may be used under the terms of the GNU Lesser General Public
 * License version 2.1 as published by the Free Software Foundation and
 * appearing in the file LICENSE.LGPL included in the packaging of this file.
 * Please review the following information to ensure the GNU Lesser General
 * Public License version 2.1 requirements will be met:
 * http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
 *----------------------------------------------------------------------------*/

/**
 * @file KalmanFilter.h
 * @author Ben Minerd
 * @date 3/28/13
 * @brief KalmanFilter class.
 */

#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

/*------------------------------------------------------------------------------
 * Include files
 *----------------------------------------------------------------------------*/

#include <Eigen/Dense>
#include <Eigen/SVD>

using Eigen::Matrix;

using namespace Eigen;

/*------------------------------------------------------------------------------
 * Classes
 *----------------------------------------------------------------------------*/

template<typename ValueType, int nStates, int nControlInputs>
class KalmanFilter // TODO : public Filter
{
public:

	/*--------------------------------------------------------------------------
	 * Public constructors and destructors
	 *------------------------------------------------------------------------*/

	KalmanFilter()
	{
	}

	/*--------------------------------------------------------------------------
	 * Public methods
	 *------------------------------------------------------------------------*/

	//--------------------------------------------------------------------------
	void predict()
	{
		// State estimate
		myXMatrix = (myFMatrix * myXMatrix) + (myBMatrix * myUMatrix);

		// Covariance estimate
		myPMatrix = (myFMatrix * myPMatrix * myFMatrix.transpose()) + myQMatrix;
	}

	//--------------------------------------------------------------------------
	void update(ValueType measurements[nStates])
	{
		Matrix<ValueType, nStates, 1> zMatrix;

		for (int i = 0; i < nStates; i++)
		{
		    zMatrix(i, 0) = measurements[i];
		}

		// Measurement
		Matrix<ValueType, nStates, 1> yMatrix = zMatrix - (myHMatrix * myXMatrix);

		// Residual covariance
		Matrix<ValueType, nStates, nStates> sMatrix =
		            (myHMatrix * myPMatrix * myHMatrix.transpose()) + myRMatrix;

		// Compute Moore-Penrose pseudo inverse of S
		ValueType pinvTolerance = 0.000001f;

		JacobiSVD<MatrixXf> sMatrixSvd(sMatrix, ComputeThinU | ComputeThinV);
		Matrix<ValueType, nStates, 1> inverseSvdValues;

		for (int i = 0; i < nStates; i++)
		{
		    if (sMatrixSvd.singularValues()(i) > pinvTolerance)
		    {
		        inverseSvdValues(i) = 1.0f / sMatrixSvd.singularValues()(i);
		    }
		    else
		    {
		        inverseSvdValues(i) = 0.0f;
		    }
		}

		Matrix<ValueType, nStates, nStates> sMatrixInverse =
		    (sMatrixSvd.matrixV() * inverseSvdValues.asDiagonal() * sMatrixSvd.matrixU().transpose());

		// Kalman gain
		Matrix<ValueType, nStates, nStates> kMatrix =
						  myPMatrix * myHMatrix.transpose() * sMatrixInverse;

		// Updated state estimate
		myXMatrix = myXMatrix + kMatrix * yMatrix;
		// Updated estimate covariance
		myPMatrix = (Matrix<ValueType, nStates, nStates>::Identity() -
				     kMatrix * myHMatrix) 							*
				    myPMatrix;
	}

	//--------------------------------------------------------------------------
	Matrix<ValueType, nStates, 1>& getXMatrix()
	{
		return myXMatrix;
	}

	//--------------------------------------------------------------------------
	Matrix<ValueType, nStates, nStates>& getFMatrix()
	{
		return myFMatrix;
	}

	//--------------------------------------------------------------------------
	Matrix<ValueType, nControlInputs, 1>& getUMatrix()
	{
		return myUMatrix;
	}

	//--------------------------------------------------------------------------
	Matrix<ValueType, nStates, nControlInputs>& getBMatrix()
	{
		return myBMatrix;
	}

	//--------------------------------------------------------------------------
	Matrix<ValueType, nStates, nStates>& getHMatrix()
	{
		return myHMatrix;
	}

	//--------------------------------------------------------------------------
	Matrix<ValueType, nStates, nStates>& getPMatrix()
	{
		return myPMatrix;
	}

	//--------------------------------------------------------------------------
	Matrix<ValueType, nStates, nStates>& getQMatrix()
	{
		return myQMatrix;
	}

	//--------------------------------------------------------------------------
	Matrix<ValueType, nStates, nStates>& getRMatrix()
	{
		return myRMatrix;
	}

private:

	/*--------------------------------------------------------------------------
	 * Private data members
	 *------------------------------------------------------------------------*/

	/**
	 * @brief X or state vector. This vector contains the current values of the
	 * filtered state variables.
	 */
	Matrix<ValueType, nStates, 1> myXMatrix;

	/**
	 * @brief F or state transition matrix (sometimes called the A matrix).
	 * @details This matrix defines how the state variables relate to each
	 * other.
	 */
	Matrix<ValueType, nStates, nStates> myFMatrix;

	/**
	 * @brief U or control input vector.
	 * @details This vector stores the control input values that will fused with
	 * the state variables.
	 */
	Matrix<ValueType, nControlInputs, 1> myUMatrix;

	/**
	 * @brief B or control matrix.
	 * @details This matrix defines how the control inputs relate to the state
	 * variables.
	 */
	Matrix<ValueType, nStates, nControlInputs> myBMatrix;

	/**
	 * @brief H or observation matrix.
	 * @details This matrix defines what state variables are physically observed
	 * or measured.
	 */
	Matrix<ValueType, nStates, nStates> myHMatrix;

	/**
	 * @brief P or predicted covariance matrix.
	 * @details This matrix contains the current covariance estimate for each
	 * state variable.
	 */
	Matrix<ValueType, nStates, nStates> myPMatrix;

	/**
	 * @brief Q or process error covariance.
	 * @details This matrix defines the process error covariance estimates for
	 * the state variables.
	 */
	Matrix<ValueType, nStates, nStates> myQMatrix;

	/**
	 * @brief R or measurement error covariance.
	 * @details This matrix defines the measurement error covariance estimates
	 * for the state variables.
	 */
	Matrix<ValueType, nStates, nStates> myRMatrix;
};

#endif // _KALMAN_FILTER_H_
