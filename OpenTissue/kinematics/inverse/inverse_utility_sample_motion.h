#ifndef OPENTISSUE_KINEMATICS_INVERSE_UTILITY_SAMPLE_MOTION_H
#define OPENTISSUE_KINEMATICS_INVERSE_UTILITY_SAMPLE_MOTION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_covariance.h>
#include <OpenTissue/core/math/math_eigen_system_decomposition.h>
#include <OpenTissue/core/math/math_compute_contiguous_angle_interval.h>


// for the matlab write function
//#include <OpenTissue/core/math/big/io/big_matlab_write.h>

#include <boost/cast.hpp>  // Needed for boost::numeric_cast

#include <cmath>   // Needed for std::fabs
#include <vector>


namespace OpenTissue
{
  namespace kinematics
  {
    namespace inverse
    {


// 2008-08-20 kenny: I think this file is obsolete, should we delete it?




      template <typename blend_scheduler_type, typename skeleton_type >
      inline std::vector < std::vector<typename skeleton_type::bone_traits::transform_type> > 
            sample_motion(blend_scheduler_type & scheduler
                          , skeleton_type & skeleton,
						  size_t N)
      {
        //size_t const N = 100u; ///< Number of samples
        typedef typename skeleton_type::bone_traits::transform_type    transform_type;
        typedef typename skeleton_type::math_types       math_types;
        typedef typename math_types::vector3_type    V;
        typedef typename math_types::real_type       T;
        typedef typename math_types::quaternion_type Q;
        typedef typename math_types::matrix3x3_type     M;
        typedef typename math_types::value_traits    value_traits;
        // Allocate space for relative transform samples
        typedef typename std::vector< transform_type > transform_container;

        std::vector< transform_container > transforms;

        transforms.resize( skeleton.size() );
        for(skeleton_type::bone_iterator bone = skeleton.begin();bone!=skeleton.end();++bone)
        {
          transforms[bone->get_number()].resize( N );;
        }


        // Sample relative transforms for each bone throughout the entire motion 

        T duration = scheduler.compute_duration();

        T time_step = duration / (N-1);
        T time = value_traits::zero();
        for(size_t i = 0;i < N;++i)
        {

          scheduler.compute_pose(skeleton, time);

          for(skeleton_type::bone_iterator bone = skeleton.begin();bone!=skeleton.end();++bone)
          {
            
            transforms[bone->get_number()][i] = bone->absolute();
          }

          time += time_step;    
        }

        return transforms;
       

      }

      

      /**
      * this version collects the entire output with all the goodies in it it is done in the bones 
      * since these have all the interface to  get to the juicy parts inside
      *
      *
      *
      **/

      template <typename blend_scheduler_type, typename skeleton_type >
      inline void 
            sample_motion_bones(
                            blend_scheduler_type & scheduler
                          , skeleton_type & skeleton 
                          , std::vector < std::vector < typename skeleton_type::bone_type> > & transforms
                          )
      {
        size_t const N = 100u; ///< Number of samples
        typedef typename skeleton_type::bone_traits::transform_type    transform_type;
        typedef typename skeleton_type::math_types       math_types;
        typedef typename math_types::vector3_type    V;
        typedef typename math_types::real_type       T;
        typedef typename math_types::quaternion_type Q;
        typedef typename math_types::matrix3x3_type     M;
        typedef typename math_types::value_traits    value_traits;
        
        
        

        // Allocate space for relative transform samples
        transforms.resize( skeleton.size() );
        for(skeleton_type::bone_iterator bone = skeleton.begin();bone!=skeleton.end();++bone)
        {
          transforms[bone->get_number()].resize( N );;
        }


        // Sample relative transforms for each bone throughout the entire motion 

        T duration = scheduler.compute_duration();

        T time_step = duration / (N-1);
        T time = value_traits::zero();
        for(size_t i = 0;i < N;++i)
        {

          scheduler.compute_pose(skeleton, time);
          OpenTissue::kinematics::inverse::set_default_joint_settings( skeleton );

          for(skeleton_type::bone_iterator bone = skeleton.begin();bone!=skeleton.end();++bone)
          {
            transforms[bone->get_number()][i] = *bone;
          }

          time += time_step;    
        }

       
       

      }

      
      /**
      * Write Sampled Data  Matlab writer for the sampled data of one bone.
      *
      * This is a utility function that is to be used in conjunction with the sample_motion
      * function.
      * it writes the sampled data to a matlab script.
      * the transforms are all coordinate systems so we will use the interface from these.
      *
      * @param prefix     A string value that is used to prefix all matlab figures. This is useful to make sure that the generated figures in the end all have unique names.
      * @param matlab_filename   The path and name of the resulting matlab script. Running this script from matlab will produce a set of figures. Also all figures are automatically exported as both eps and png files.
      * @param latex_filename    The path and name of the resulting latex file. Upon return this file will contain latex code for some of the statistics.
      * @param begin             An iterator to the position of the first Output class.
      * @param end               An iterator to the position one past the last Output class.
      **/
	  /*
      template<typename pose_iterator> 
      inline void write_sampled_transforms( 
                                std::string const & prefix
                              , std::string const & matlab_filename
                              , std::string const & latex_filename
                              , pose_iterator begin
                              , pose_iterator end)
      {
          using namespace OpenTissue::math::big;
          using std::min;
          using std::max;

          std::ofstream matlab_file( matlab_filename.c_str(), std::ios::out);
          if (!matlab_file)
          {
            std::cerr << "Error unable to create file: "<< matlab_filename << std::endl;
            return;
          }

          std::ofstream latex_file( latex_filename.c_str(), std::ios::out);
          if (!latex_file)
          {
            std::cerr << "Error unable to create file: "<< latex_filename << std::endl;
            return;
          }



          // Make sure that matlab environment is ready
          matlab_file << "close all;" << std::endl;
          matlab_file << "clear all;" << std::endl;


            //make a vector of the phi samples
            
            matlab_file << "phi = [";
            for(pose_iterator output=begin;output!=end;++output)
            {
              matlab_file << " " <<  output->theta(0);
            }
            matlab_file << " ];" << std::endl;

            //make a vector of the psi samples
             matlab_file << "psi = [";
            for(pose_iterator output=begin;output!=end;++output)
            {
              matlab_file << " " <<  output->theta(1);
            }
            matlab_file << " ];" << std::endl;
            //make a vector of the theta samples
             matlab_file << "theta = [";
            for(pose_iterator output=begin;output!=end;++output)
            {
              matlab_file << " " <<  output->theta(2);
            }
            matlab_file << " ];" << std::endl;
          
            // write some helpful values to the prompt should be changed so they are put in some figure of sorts
           matlab_file << "min_phi = min(phi)" << std::endl;
           matlab_file << "max_phi = max(phi)" << std::endl;
           matlab_file << "min_psi = min(psi)" << std::endl;
           matlab_file << "max_psi = max(psi)" << std::endl;
           matlab_file << "min_theta = min(theta)" << std::endl;
           matlab_file << "max_theta = max(theta)" << std::endl;

     // Create  plots of phi psi and theta
          {
            matlab_file << "filename1 = '" << prefix << "_parameters';" << std::endl;
            matlab_file << "figure(1);" << std::endl;
            matlab_file << "clf" << std::endl;
            matlab_file << "set(gca,'fontsize',18);" << std::endl;
            matlab_file << "hold on;" << std::endl;
            
            matlab_file << "plot(phi,'r');" << std::endl;
            matlab_file << "plot(psi,'g');" << std::endl;
            matlab_file << "plot(theta,'b');" << std::endl;    
            matlab_file << "legend('phi ','psi ','theta','Location','Best');" << std::endl;    
            matlab_file << "axis tight;" << std::endl;
            matlab_file << "xlabel('Samples','fontsize',18);" << std::endl;
            matlab_file << "ylabel('Value of the parameters','fontsize',18);" << std::endl;
            matlab_file << "hold off;" << std::endl;
            matlab_file << "print('-f1','-depsc2', filename1);" << std::endl;
            matlab_file << "print('-f1','-dpng', filename1);" << std::endl;
          }



          
          matlab_file.flush();
          matlab_file.close();

          latex_file.flush();
          latex_file.close();
          std::cout << "done writting sampled datafiles..." << std::endl;
      
      }*/
    }// namespace inverse
  } //namespace kinematics
}//namespace OpenTissue


//OPENTISSUE_KINEMATICS_INVERSE_UTILITY_SAMPLE_MOTION_H
#endif
