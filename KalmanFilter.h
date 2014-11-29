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

template<int nStates, int nControlInputs>
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
	void update(float measurements[nStates])
	{
		Matrix<float, nStates, 1> zMatrix;

		for (int i = 0; i < nStates; i++)
		{
		    zMatrix(i, 0) = measurements[i];
		}

		// Measurement
		Matrix<float, nStates, 1> yMatrix = zMatrix - (myHMatrix * myXMatrix);

		// Residual covariance
		Matrix<float, nStates, nStates> sMatrix =
		            (myHMatrix * myPMatrix * myHMatrix.transpose()) + myRMatrix;

		// Compute Moore-Penrose pseudo inverse of S
		float pinvTolerance = 0.000001f;

		JacobiSVD<MatrixXf> sMatrixSvd(sMatrix, ComputeThinU | ComputeThinV);
		Matrix<float, nStates, 1> inverseSvdValues;

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

		Matrix<float, nStates, nStates> sMatrixInverse =
		    (sMatrixSvd.matrixV() * inverseSvdValues.asDiagonal() * sMatrixSvd.matrixU().transpose());

		// Kalman gain
		Matrix<float, nStates, nStates> kMatrix =
						  myPMatrix * myHMatrix.transpose() * sMatrixInverse;

		// Updated state estimate
		myXMatrix = myXMatrix + kMatrix * yMatrix;
		// Updated estimate covariance
		myPMatrix = (Matrix<float, nStates, nStates>::Identity() -
				     kMatrix * myHMatrix) 							*
				    myPMatrix;
	}

	//--------------------------------------------------------------------------
	Matrix<float, nStates, 1>& getXMatrix()
	{
		return myXMatrix;
	}

	//--------------------------------------------------------------------------
	Matrix<float, nStates, nStates>& getFMatrix()
	{
		return myFMatrix;
	}

	//--------------------------------------------------------------------------
	Matrix<float, nControlInputs, 1>& getUMatrix()
	{
		return myUMatrix;
	}

	//--------------------------------------------------------------------------
	Matrix<float, nStates, nControlInputs>& getBMatrix()
	{
		return myBMatrix;
	}

	//--------------------------------------------------------------------------
	Matrix<float, nStates, nStates>& getHMatrix()
	{
		return myHMatrix;
	}

	//--------------------------------------------------------------------------
	Matrix<float, nStates, nStates>& getPMatrix()
	{
		return myPMatrix;
	}

	//--------------------------------------------------------------------------
	Matrix<float, nStates, nStates>& getQMatrix()
	{
		return myQMatrix;
	}

	//--------------------------------------------------------------------------
	Matrix<float, nStates, nStates>& getRMatrix()
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
	Matrix<float, nStates, 1> myXMatrix;

	/**
	 * @brief F or state transition matrix (sometimes called the A matrix).
	 * @details This matrix defines how the state variables relate to each
	 * other.
	 */
	Matrix<float, nStates, nStates> myFMatrix;

	/**
	 * @brief U or control input vector.
	 * @details This vector stores the control input values that will fused with
	 * the state variables.
	 */
	Matrix<float, nControlInputs, 1> myUMatrix;

	/**
	 * @brief B or control matrix.
	 * @details This matrix defines how the control inputs relate to the state
	 * variables.
	 */
	Matrix<float, nStates, nControlInputs> myBMatrix;

	/**
	 * @brief H or observation matrix.
	 * @details This matrix defines what state variables are physically observed
	 * or measured.
	 */
	Matrix<float, nStates, nStates> myHMatrix;

	/**
	 * @brief P or predicted covariance matrix.
	 * @details This matrix contains the current covariance estimate for each
	 * state variable.
	 */
	Matrix<float, nStates, nStates> myPMatrix;

	/**
	 * @brief Q or process error covariance.
	 * @details This matrix defines the process error covariance estimates for
	 * the state variables.
	 */
	Matrix<float, nStates, nStates> myQMatrix;

	/**
	 * @brief R or measurement error covariance.
	 * @details This matrix defines the measurement error covariance estimates
	 * for the state variables.
	 */
	Matrix<float, nStates, nStates> myRMatrix;
};

#endif // _KALMAN_FILTER_H_
