#ifndef OPENTISSUE_KINEMATICS_INVERSE_INVERSE_BONE_TRAITS_H
#define OPENTISSUE_KINEMATICS_INVERSE_INVERSE_BONE_TRAITS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_euler_angles.h> // needed for ZYZ_euler_angles

#include <cassert>
#include <math.h>

namespace OpenTissue
{
  namespace kinematics
  {
    namespace inverse
    {
      
      /**
       * Default Bone Traits for Inverse Kinematics.
       * This class provides functionality to handle constrained inverse kinematics.
       *
       * @warning If end-users wants to extend with their own bone traits
       *          then their bone traits must comply with a similar interface
       *          as this bone traits class has.
       *
       * @tparam  base_bone_traits     A base bone traits. This could for instance be the default
       *                               bone traits from the skeleton library. This traits class must
       *                               define the types of transforms and different ways of manipulating
       *                               and doing calculations with these types. Thus this bone trait
       *                               class does not care about what kind of coordinate representation
       *                               one wants to use for the skeleton/bone transformations.
       */
      template<typename base_bone_traits >
      class BoneTraits 
        : public base_bone_traits
        {
        public:
          
          typedef typename base_bone_traits::transform_type               transform_type;
          typedef typename base_bone_traits::vector3_type                 vector3_type;
		  typedef typename transform_type::matrix3x3_type                 matrix_type;
          typedef typename base_bone_traits::math_types::real_type        real_type;
          typedef typename base_bone_traits::math_types::value_traits     value_traits;
          
          typedef enum {hinge_type, slider_type, ball_type}               joint_type;
          
        protected:
          
          joint_type            m_type;              ///< The joint type that this bone is connected to its parent bone with.
          real_type             m_theta[6];          ///< Joint parameter values.
          real_type             m_min_theta[6];      ///< Minimum joint parameter value.
          real_type             m_max_theta[6];      ///< Maximum joint parameter value.
          vector3_type          m_u;                 ///< Unit joint axis, given as a constant vector in parent frame.
          
        public:

			transform_type pivot;
			transform_type inv_pivot;
			transform_type parent_pivot;
			
			bool update_theta;
          
          BoneTraits()
          : m_type(ball_type)
          , m_u( value_traits::zero(), value_traits::zero(), value_traits::one() )
          {
            // Create some default values
            for(size_t i=0;i<6;++i)
            {
              m_theta[i] = value_traits::zero();
              m_min_theta[i] = -value_traits::pi();
              m_max_theta[i] =  value_traits::pi();    
            }
            
            
          }
          
          /** 
           * Get Joint Type.
           *
           * @return   A reference to the joint type of this bone.
           */
          joint_type const & type() const  { return this->m_type; }
          joint_type       & type()        { return this->m_type; }
          
          /**
           * Get Joint Axis.
           *
           * @return   A reference to the joint axis. This is used
           *           for specifying the motion of slider and revolute
           *           joint types. It is un-needed for ball type joints.
           */
          vector3_type const & u() const  { return this->m_u; }
          vector3_type       & u()        { return this->m_u; }
          
          /**
           * Get Joint Parameter Value.
           *
           * @param i     The index of the joint paramter value that is wanted.
           * @return      A reference to the corresponding joint parameter value.
           */
          real_type const & theta(size_t const & i) const 
          {
            assert( i < this->active_dofs() || !"theta(): invalid index value");
            return m_theta[i];
          }
          
          /**
           * Get Joint Parameter Value.
           *
           * @param i     The index of the joint parameter value that is wanted.
           * @return      A reference to the corresponding joint parameter value.
           */
          real_type      & theta(size_t const & i)
          {
            assert( i < this->active_dofs() || !"theta(): invalid index value");
            return m_theta[i];
          }
          
          /**
           * Get number of Active Degrees of Freedon (DOFs).
           * This method computes and returns the number of active degrees
           * of freedom for the bone. This number is related directly to the
           * specified joint type of the bone.
           *
           * @return The number of active degrees of freedom of the bone.
           */
          size_t active_dofs() const 
          {
            if(this->m_type==hinge_type)
              return 1;
            if(this->m_type==slider_type)
              return 1;
            if(this->m_type==ball_type)
              return 3;
            return 0;
          }
          
          /**
           * Get Minimum Joint Limit.
           *
           * @param i   The index of the value for which the limits is wanted.
           * @return    A reference to the corresponding joint limit.  
           */
          real_type const & min_joint_limit(size_t const & i) const 
          {
            assert( i < this->active_dofs() || !"min_joint_limit(): invalid index value");
            return m_min_theta[i];
          }
          
          /**
           * Get Minimum Joint Limit.
           *
           * @param i   The index of the value for which the limits is wanted.
           * @return    A reference to the corresponding joint limit.  
           */
          real_type      & min_joint_limit(size_t const & i)
          {
            assert( i < this->active_dofs() || !"min_joint_limit(): invalid index value");
            return m_min_theta[i];
          }
          
          /**
           * Get Maximum Joint Limit.
           *
           * @param i   The index of the value for which the limits is wanted.
           * @return    A reference to the corresponding joint limit.  
           */
          real_type const & max_joint_limit(size_t const & i) const 
          {
            assert( i < this->active_dofs() || !"max_joint_limit(): invalid index value");
            return m_max_theta[i];
          }
          
          /**
           * Get Maximum Joint Limit.
           *
           * @param i   The index of the value for which the limits is wanted.
           * @return    A reference to the corresponding joint limit.  
           */
          real_type      & max_joint_limit(size_t const & i)
          {
            assert( i < this->active_dofs() || !"max_joint_limit(): invalid index value");
            return m_max_theta[i];
          }
          
          /**
           * Computes the Jacobian matrix corresponding to this bone
           *
           * @Param bone     The bone for which the Jacobian is to be calculated
           * @Param chain    The corresponding chain for specified bone.
           * @Param J        Upon return this holds the bub-block of the Jacobian
           *                 corresponding to the specified bone and end-effector
           *                 of the corresponding chain.
           */
          template<typename bone_type, typename chain_type, typename matrix_range>
          static void compute_jacobian( bone_type & bone, chain_type & chain, matrix_range & J )
          {
            assert(J.size1() == chain.get_goal_dimension() || !"compute_jacobian() invalid dimension");
            assert(J.size2() == bone.active_dofs()         || !"compute_jacobian() invalid dimension");
            
            // Get tool frame in world coordinate system
            vector3_type p = base_bone_traits::transform_point(  chain.get_end_effector()->absolute(),  chain.p_local() );

            p = p - bone.absolute().T();// new term that was missing and may also be in the article

            vector3_type i = unit(base_bone_traits::transform_vector( chain.get_end_effector()->absolute(),  chain.x_local() ));
            vector3_type j = unit(base_bone_traits::transform_vector( chain.get_end_effector()->absolute(),  chain.y_local() ));
            
            if(bone.type()==hinge_type)
            {
              // Get joint axis in world coordinate frame
              vector3_type u = bone.u(); // If bone is root then joint axis is already in world coordinate frame
              if(bone.parent())
              {
                u = base_bone_traits::transform_vector( bone.parent()->absolute(),  bone.u() );
              }
              
              vector3_type uXp = cross(u, p);
              
              J(0,0) = uXp(0);
              J(1,0) = uXp(1);
              J(2,0) = uXp(2);
              
              if(!chain.only_position() )// we need to do the orientation derivatives also
              {
                vector3_type uXi = cross(u, i);
                vector3_type uXj = cross(u, j);
                
                J(3,0) = uXi(0);
                J(4,0) = uXi(1);
                J(5,0) = uXi(2);
                
                J(6,0) = uXj(0);
                J(7,0) = uXj(1);
                J(8,0) = uXj(2);
              }
            }
            else if(bone.type()==slider_type)
            {
              // Get joint axis in world coordinate frame
              vector3_type u = bone.u(); // If bone is root then joint axis is already in world coordinate frame
              if(bone.parent())
              {
                u = base_bone_traits::transform_vector( bone.parent()->absolute(),  bone.u() );
              }
              
              J(0,0) = u(0);
              J(1,0) = u(1);
              J(2,0) = u(2);
              
              if(!chain.only_position() )// we need to do the orientation derivatives also
              {
                J(3,0) = value_traits::zero();
                J(4,0) = value_traits::zero();
                J(5,0) = value_traits::zero();
                J(6,0) = value_traits::zero();
                J(7,0) = value_traits::zero();
                J(8,0) = value_traits::zero();
              }
            }
            else if(bone.type()==ball_type)
            {
              vector3_type ii = vector3_type(value_traits::one(),value_traits::zero(),value_traits::zero());
              vector3_type jj = vector3_type(value_traits::zero(),value_traits::one(),value_traits::zero());
              vector3_type kk = vector3_type(value_traits::zero(),value_traits::zero(),value_traits::one());
              
              // Extract joint angles
              real_type phi   = bone.theta(0);
              real_type psi   = bone.theta(1);
              real_type theta = bone.theta(2);

			  //transform_type::matrix3x3_type rotation(bone.rotation_from_euler(vector3_type(phi, psi, theta), bone));
              //bone.parent_pivot.Q() % fromEuler % bone.inv_pivot.Q()

			  matrix_type zm = OpenTissue::math::Rx(-theta);
			  matrix_type ym = OpenTissue::math::Ry(-psi);
			  matrix_type ofp(bone.parent_pivot.Q());

              // Compute the instantaneous axis of rotation (in parent frame)
			  vector3_type u = ofp * zm * ym * -ii;
			  vector3_type v = ofp * zm * -jj;
			  vector3_type w = ofp * -kk;
			  //vector3_type v = matrix_type(bone.parent_pivot.Q()) * OpenTissue::math::Rx(phi) * jj;//rotation.rotate(jj);
			  //vector3_type w = matrix_type(bone.parent_pivot.Q()) * OpenTissue::math::Rx(phi) * OpenTissue::math::Ry(psi) * kk;//rotation.rotate(kk);
              
              // Transform instantaneous rotation axes into world coordinate frame
              if(bone.parent())
              {
				  vector3_type tmp = base_bone_traits::transform_vector(bone.parent()->absolute(),  u );
                u = unit(tmp);
                tmp = base_bone_traits::transform_vector( bone.parent()->absolute(),  v );
                v = unit(tmp);
                tmp = base_bone_traits::transform_vector( bone.parent()->absolute(),  w );
                w = unit(tmp);
              }
              
              vector3_type uXp = cross(u, p);
              vector3_type vXp = cross(v, p);
              vector3_type wXp = cross(w, p);
             
              // setup phi part
              J(0,0) = uXp(0);
              J(1,0) = uXp(1);
              J(2,0) = uXp(2);
              
              // setup psi part
              J(0,1) = vXp(0);
              J(1,1) = vXp(1);
              J(2,1) = vXp(2);
              
              // setup theta part
              J(0,2) = wXp(0);
              J(1,2) = wXp(1);
              J(2,2) = wXp(2);
              
              if( ! chain.only_position() ) // we need to do the orientation derivatives also
              {
                vector3_type uXi = cross(u, i);
                vector3_type uXj = cross(u, j);
                
                vector3_type vXi = cross(v, i);
                vector3_type vXj = cross(v, j);
                
                vector3_type wXi = cross(w, i);
                vector3_type wXj = cross(w, j);
                
                J(3,0) = uXi(0);
                J(4,0) = uXi(1);
                J(5,0) = uXi(2);
                
                J(6,0) = uXj(0);
                J(7,0) = uXj(1);
                J(8,0) = uXj(2);
                
                J(3,1) = vXi(0);
                J(4,1) = vXi(1);
                J(5,1) = vXi(2);
                
                J(6,1) = vXj(0);
                J(7,1) = vXj(1);
                J(8,1) = vXj(2);
                
                J(3,2) = wXi(0);
                J(4,2) = wXi(1);
                J(5,2) = wXi(2);
                
                J(6,2) = wXj(0);
                J(7,2) = wXj(1);
                J(8,2) = wXj(2);
              }
            }
            else
            {
              assert(false || !"compute_jacobian(): unknown joint type");
            }
          }
		  
		  template<typename bone_type>
		  static void set_theta(bone_type &bone, real_type t1, real_type t2, real_type t3)
		  {
			bone.theta(0) = t1;
			bone.theta(1) = t2;
			bone.theta(2) = t3;
		  }
          
          /**
           * Set Joint Parameter Values.
           * This method reads the joint parameter values from a specified
           * sub-vector. The function copies the values into the local storage
           * in this bone and further it sets the relative transformation of the
           * bone to match the joint parameter values.
           *
           * It is rather important that the relative transformation of the bone
           * is updated accordingly otherwise other libraries like skinning and
           * animation will stop working correctly when used together with
           * the inverse kinematics library.
           *
           * @warning  Observe that after having set the joint parameter values of all
           *           bones then one should also re-compute the absolute bone transformations
           *           if these are needed. The inverse kinematics library will do this by
           *           default.
           *
           *
           * @param bone             The bone that should be worked on. 
           * @param sub_theta        A vector or vector range from which the joint
           *                         parameters should be extracted.
           */
          template<typename bone_type, typename vector_range>
          static void set_theta(bone_type & bone, vector_range const & sub_theta)
          {
            assert(sub_theta.size() == bone.active_dofs() || !"set_theta() invalid dimension");
            
            if(bone.type() == hinge_type)
            {
              bone.theta(0) = sub_theta(0);
              bone.relative().Q() = OpenTissue::math::Ru(bone.theta(0), bone.u() );
            }
            else if(bone.type() == slider_type)
            {
              bone.theta(0) = sub_theta(0);
              bone.relative().T() = bone.theta(0) * bone.u();
            }
            else if(bone.type() == ball_type)
            {
              real_type phi   = (bone.theta(0) = sub_theta(0) );
              real_type psi   = (bone.theta(1) = sub_theta(1) );
              real_type theta = (bone.theta(2) = sub_theta(2) );
              
              bone.relative().Q() = bone.rotation_from_euler(vector3_type(phi, psi, theta), bone);
			  //if(!bone.is_root())
				//bone.relative().T() = base_bone_traits::transform_vector( bone.parent()->absolute(), bone.pivot).T();
			  //if(!bone.is_root())
				//bone.absolute() = bone_type::compute_absolute_pose_transform(bone.parent()->absolute(), bone.relative());
			  //else
				 //bone.absolute() = bone.relative();
            }
            else
            {
              assert(false || !"set_theta(): unknown joint type");
            }
          }

		  // Added for LabMan: convert Euler angles (XYZ) into quaternion
		  template<typename bone_type>
		inline typename transform_type::quaternion_type rotation_from_euler(const vector3_type &angles, const bone_type &bone)
		  {
			  //return quaternion_type(OpenTissue::math::Rx(angles[0]) * OpenTissue::math::Ry(angles[1]) * OpenTissue::math::Rz(angles[2]));
			  transform_type::quaternion_type fromEuler = transform_type::quaternion_type(OpenTissue::math::Rx(angles[0]) * OpenTissue::math::Ry(angles[1]) * OpenTissue::math::Rz(angles[2]));
			  fromEuler = OpenTissue::math::conj(fromEuler);
			  if(bone.is_root())
			  {
				  fromEuler = fromEuler % bone.bind_pose().Q();
				  return fromEuler;
			  }

			  fromEuler = bone.parent_pivot.Q() % fromEuler % bone.inv_pivot.Q();
			  return fromEuler;

			  //return absoluteToRelative(fromEuler, bone.parent()->absolute());
		  }
		  inline static typename transform_type::quaternion_type absoluteToRelative(const typename transform_type::quaternion_type &absolute, const typename transform_type::quaternion_type &parent)
		  {
			  typename transform_type::quaternion relative;

			//relative.T() = OpenTissue::math::inverse(parent).Q().rotate(absolute.T() - parent.T());
			relative = parent % absolute;

			return relative;
		  }
          
          /**
           * Extract Joint Parameter Values of a Bone.
           *
           * @param bone             The bone that should be worked on.
           * @param sub_theta        A vector or vector range that upon return holds the joint
           *                         parameter values of the specified bone.
           */
          template<typename bone_type, typename vector_range>
          static void get_theta(bone_type const & bone, vector_range & sub_theta)
          {
            assert(sub_theta.size() == bone.active_dofs()         || !"get_theta() invalid dimension");
            
            if(bone.type() == hinge_type)
            {
              sub_theta(0) = bone.theta(0);
            }
            else if(bone.type() == slider_type)
            {
              sub_theta(0) = bone.theta(0);
            }
            else if(bone.type() == ball_type)
            {
              sub_theta(0) = bone.theta(0);
              sub_theta(1) = bone.theta(1);
              sub_theta(2) = bone.theta(2);
            }
            else
            {
              assert(false || !"get_theta(): unknown joint type");
            }
          }
          
          /**
           * Extract joint limits of a Bone.
           *
           * @param bone             The bone that should be worked on.
           * @param sub_min          A vector or vector range that upon return holds the minimum joint
           *                         limits of the specified bone.
           * @param sub_max          A vector or vector range that upon return holds the maximum joint
           *                         limits of the specified bone.
           */
          template<typename bone_type, typename vector_range>
          static void get_joint_limits(bone_type const & bone, vector_range & sub_min,vector_range & sub_max)
          {
            assert(sub_min.size() == bone.active_dofs()         || !"get_joint_limits() invalid dimension");
            assert(sub_max.size() == bone.active_dofs()         || !"get_joint_limits() invalid dimension");
            
            if(bone.type() == hinge_type)
            {
              sub_min(0) = bone.min_joint_limit(0);
              sub_max(0) = bone.max_joint_limit(0);
            }
            else if(bone.type() == slider_type)
            {
              sub_min(0) = bone.min_joint_limit(0);
              sub_max(0) = bone.max_joint_limit(0);
            }
            else if(bone.type() == ball_type)
            {
              sub_min(0) = bone.min_joint_limit(0);
              sub_min(1) = bone.min_joint_limit(1);
              sub_min(2) = bone.min_joint_limit(2);
              
              sub_max(0) = bone.max_joint_limit(0);
              sub_max(1) = bone.max_joint_limit(1);
              sub_max(2) = bone.max_joint_limit(2);
            }
            else
            {
              assert(false || !"get_joint_limits(): unknown joint type");
            }
          }
          
		  static inline void getXYZ(const typename transform_type::matrix3x3_type &mat, vector3_type &angles)
		  {
		    if (mat[0][2] < 1.0)
			{
				if (mat[0][2] > -1.0)
				{
                     angles[0] = atan2(-mat[1][2],mat[2][2]);
                     angles[1] = asin(mat[0][2]);
                     angles[2] = atan2(-mat[0][1],mat[0][0]);
	            }
				else
				{
                     // WARNING.  Not unique.  XA - ZA = -atan2(r10,r11)
					angles[0] = -atan2(mat[1][0],mat[1][1]);
					angles[1] = -value_traits::pi_half();
                     angles[2] = 0.0;
				}
			}
			else
			{
             // WARNING.  Not unique.  XAngle + ZAngle = atan2(r10,r11)
             angles[0] = atan2(mat[1][0],mat[1][1]);
			 angles[1] = value_traits::pi_half();
             angles[2] = 0.0;

			}

		  } // getXYZ()

          /**
           * Project sub-vector onto joint limits of a Bone.
           *
           * @warning    Note that the joint parameter values are not extracted
           *             from the bone, rather it is assumed that the specified
           *             sub-vector contains the current joint paramter values 
           *             that should be used.
           *
           * @param bone             The bone that should be worked on.
           * @param sub_theta        A vector or vector range that initially holds the current joint
           *                         parameter values of the specified bone and which upon return
           *                         holds the projected values.
           */
          template<typename bone_type, typename vector_range>
          static void joint_limit_projection( bone_type const & bone, vector_range & sub_theta)
          {
            using std::max;
            using std::min;
            
            assert(sub_theta.size() == bone.active_dofs() || !"joint_limit_projection() invalid dimension");
            assert(sub_theta.size() == bone.active_dofs() || !"joint_limit_projection() invalid dimension");
            
            if(bone.type() == hinge_type || bone.type() == slider_type)
            {
              sub_theta(0) = max( bone.min_joint_limit(0), min(bone.max_joint_limit(0), sub_theta(0) ) );
            }
            else if(bone.type() == ball_type)
            {
              sub_theta(0) = max( bone.min_joint_limit(0), min(bone.max_joint_limit(0), sub_theta(0)) );
			  sub_theta(1) = max( bone.min_joint_limit(1), min(bone.max_joint_limit(1), sub_theta(1)) );
			  sub_theta(2) = max( bone.min_joint_limit(2), min(bone.max_joint_limit(2), sub_theta(2)) );
            }
            else
            {
              assert(false || !"joint_limit_projection(): unknown joint type");
            }
          }
          
          
        };
      
      
      
      // 2009-03-26 kenny: I am not sure the name of the function reflects what it is doing?
      // 2009-03-26 kenny: The documentation is not in alignment with the implementation. In the implementation you use the relative transform, not the absolute transform. Thus, there should be no need for calling compute_poses().
      // 2009-03-26 kenny: The documentation is not in alignment with the implementation. I am not sure I understand the use-pattern, if one has performed forward kinematics, exactly what do you mean?
      /**
       * Synchronize Bone. 
       * Synchronize makes sure the theta vector of a bone is equivalent to the actual
       * pose of the bone. It uses the absolute pose of the bone for this so it is important 
       * to make sure this is updated e.g. by running compute_pose() on the skeleton. It is 
       * useful if one has performed forward kinematics on an inverse kinematics skeleton 
       * The computations are the same as in the function set default joint parameters except 
       * this function uses the absolute pose while the other one uses bind_pose.
       *
       * @param  bone  The bone which should be synchronized
       *
       */
      // 2009-03-26 kenny: Where is the unit-test for this free template function?
      template<typename bone_type>
      inline void synchronize_bone(bone_type & bone)
      {
        using std::atan2;
        
        typedef typename bone_type::bone_traits             bone_traits;
        typedef typename bone_traits::transform_type        transform_type;
        typedef typename bone_traits::real_type             real_type;
        typedef typename bone_traits::value_traits          value_traits;
        typedef typename bone_traits::vector3_type          vector3_type;
        typedef typename transform_type::quaternion_type    quaternion_type;
        
        transform_type const & T = bone.relative();
        
        if(bone.type() == bone_traits::hinge_type)
        {
          quaternion_type Q = T.Q();
          vector3_type u = unit( Q.v() );
          // we know
          //
          // Q = [cos(theta/2), sin(theta/2) u]
          //
          real_type const ct2    = Q.s();           //---   cos(theta/2)
          real_type const st2    = length( Q.v() ); //---  |sin(theta/2)|
          real_type const theta  = value_traits::two()* atan2(st2,ct2);
          // of course it might have been -|sin(theta/2)| that were the correct solution?
          bone.u()      = u;
          bone.theta(0) = theta;
        }
        else if(bone.type() == bone_traits::slider_type)
        {
          vector3_type const u     = unit( T.T() );
          real_type    const theta = length( T.T() );
          bone.u()      = u;
          bone.theta(0) = theta;
        }
        else if(bone.type() == bone_traits::ball_type)
        {
          real_type phi     = value_traits::zero();
          real_type psi     = value_traits::zero();
          real_type theta   = value_traits::zero();
        
          quaternion_type Q = T.Q();
        
          OpenTissue::math::ZYZ_euler_angles(Q, phi, psi, theta);
      
          // Now we can store the correct joint parameter values into the bone
          bone.theta(0) = phi;
          bone.theta(1) = psi;
          bone.theta(2) = theta;
        }
      }
      
      
    } // namespace inverse
  } // namespace kinematics
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_INVERSE_INVERSE_BONE_TRAITS_H
#endif
