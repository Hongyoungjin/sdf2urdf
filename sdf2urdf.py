import numpy as np
import os
from math import pi

class sdf2urdf:

    def __init__(self):

        self.sdf_path  = '/path/to/sdf/model.sdf'
        self.urdf_path = '/path/to/urdf.urdf'

        self.sdf = self.sdf_refine(self.sdf_path)

    # Filter out meaningless strings
    def sdf_refine(self,path):

        sdf = open(path,'r')
        sdf = sdf.readlines()
        sdf = [line.strip() for line in sdf]
        sdf = np.array(sdf)

        # Remove comments to prevent adverse convertion
        nodes = self.find_nodes(sdf,'<!--','-->','generous','generous')
        comment_indices = np.zeros(1)

        for node in nodes:
            comment_index = np.arange(int(node[1]-node[0]+1)) + int(node[0])
            comment_indices = np.append(comment_indices,comment_index)
            
        comment_indices = list(comment_indices[1:].astype(int))
        sdf = np.delete(sdf,comment_indices)
        return sdf

    # Find the file path by its name
    def find(self,name,path):
        for root, dirs, files in os.walk(path):
            if name in dirs:
                return os.path.join(root, name)
    
    def purge(self,node_list):
        joined_list = []

        for node in node_list:
            joined_node = []
            for lines in node:
                # Concatenate several node lines into a line
                joined_line = ''.join(lines)
                joined_node.append(joined_line)
            # Concatenate all lines of a node into one sentence
            joined_node = ''.join(joined_node)
            joined_list.append(joined_node)
        # Eliminate all node duplicates
        uniques = list(dict.fromkeys(joined_list))
        return uniques

    # Find any element that contains certain keyword
    def find_index(self,sdf,keyword):

        elements = [s for s in sdf if keyword in s]
        indices = np.zeros(1)

        for element in elements:
            index = np.where(sdf == element)
            indices = np.append(indices,index)

        indices = indices[1:]

        return indices

    # Find where </link name ..> ... </link> or </joint name .. > ... </joint> is
    def find_nodes(self,sdf,keyword_start,keyword_end,start_option,end_option):

        nodes = np.zeros((1,2))
        if start_option == 'finicky': # Used when finding <include> statements
            indices_start = np.where(sdf == keyword_start)[0]

        if start_option == 'generous': # Used when finding <link> & <joint> statements
            indices_start = self.find_index(sdf,keyword_start)

        if end_option == 'finicky': # Used to find link and joint statements
            indices_end   = np.where(sdf == keyword_end)[0]

        if end_option == 'generous':  # Used when removing comments
            indices_end   = self.find_index(sdf,keyword_end)

        # Group two sequential elements; first is starting index, and second is ending index
        for i in range(len(indices_start)):
            node = np.array([indices_start[i],indices_end[i]]).reshape((1,2))
            nodes = np.append(nodes,node,axis=0)

        return nodes[1:]

    # Convert </robot>
    def convert_robot(self):

        model_start = [s for s in self.sdf if 'model name=' in s][0]
        robot_start = model_start.replace("model","robot") 
        
        return robot_start + '\n'
    
    # convert </link> to </link>
    def convert_link(self,sdf):

        nodes = self.find_nodes(sdf,'link name=','</link>','generous','finicky')
        link_urdf = []

        for node in nodes: # Convert in every link statement
            
            link = sdf[int(node[0]) : int(node[1])+1]

            # Catch link name
            link_name = str(link[0])
            # Catch visual name
            visual = link_name.replace('link','visual')

            # Catch mesh from mesh path
            mesh = [s for s in link if '<uri>' in s]
            if len(mesh)>0:
                mesh = str(mesh[0])
                mesh = mesh.replace("<uri>","")
                mesh = mesh.replace("</uri>","")
                mesh = mesh.replace("model","package")
                mesh = "'" + mesh + "'"

            # Catch mesh from figure description
            else:
                box      = [s for s in link if '<box>' in s]
                if len(box)>0:
                    size = str([s for s in link if '<size>' in s][0])
                    size = size.replace("<size>","")
                    size = size.replace("</size>","")

                    fig = "<box size=" + "'" + size +  "'/>\n"

                cylinder = [s for s in link if '<cylinder>' in s]
                if len(cylinder)> 0:
                    radius = str([s for s in link if '<radius>' in s][0])
                    radius = radius.replace("<radius>","")
                    radius = radius.replace("</radius>","")

                    length = str([s for s in link if '<length>' in s][0])
                    length = length.replace("<length>","")
                    length = length.replace("</length>","")

                    fig = "<cylinder radius=" + "'" + radius + "' length ='" + length +"'/>\n"

                sphere   = [s for s in link if '<sphere>' in s]
                if len(sphere) > 0:
                    radius = str([s for s in link if '<radius>' in s][0])
                    radius = radius.replace("<radius>","")
                    radius = radius.replace("</radius>","")

                    fig = "<sphere radius=" + "'" + radius +  "'/>\n"

            # Catch material
            try: 
                material = str([s for s in link if '<material>' in s][0])
                color    = str([s for s in link if '<emissive>' in s][0])
                color    = color.replace('<emissive>','')
                color    = color.replace('</emissive>','')
                color    = "<material name='noname'>" + '\n'+ "    <color rgba='" + color + "'/>"+"\n" + '</material>' +'\n'

            except:
                color = ''
                pass

            # Catch scale
            try: 
                scale = str([s for s in link if '<scale>' in s][0])
                scale = scale.replace("<scale>","")
                scale = scale.replace("</scale>","")
                scale = " scale='" + scale  + "'"
            except:
                scale = ''
                pass

            # Catch pose > origin
            pose = [s for s in link if '<pose>' in s]
            if len(pose)>0:
                pose = str(pose[0])
                pose = pose.replace("<pose>","")
                pose = pose.replace("</pose>","")
                x,y,z,= pose.split()[0:3]
                roll,pitch,yaw = pose.split()[3:6]
                origin = "xyz='" + x + " " + y + " " + z + "' rpy='" + roll + " " + pitch + " " + yaw + " " + "'"
            else:
                origin = ''
            
            # Rewrite in urdf form 
            link1 = [
            link_name + '\n',
            ' ' + visual + "\n",
            '  <origin ' + origin + '/>\n',
            '  <geometry>\n']
            if len(mesh)>0:
                link2 = ['   <mesh filename =' + mesh + scale + '/>\n']
            else:
                link2 = [fig]
            link3 = ['  </geometry>\n']
            link4 = [color]
            link5 = [
            ' </visual>\n',
            '</link>\n\n']
            link_temp = []
            link_temp.append(link1)
            link_temp.append(link2)
            link_temp.append(link3)
            link_temp.append(link4)
            link_temp.append(link5)

            link_urdf.append(link_temp)

        return link_urdf

    # convert </include> to </link>
    def convert_include(self):

        nodes = self.find_nodes(self.sdf,'<include>','</include>','finicky','finicky')
        include_urdf = []

        for node in nodes: # Convert in every include statement
            
            include = self.sdf[int(node[0]) : int(node[1])+1]
            # Catch name
            name = str([s for s in include if '<name>' in s][0])
            name = name.replace("<name>","")
            name = name.replace("</name>","")

            # Define link name 
            link_name   = '<link name=' + "'" + name + "'" + '>'

            # Define visual name
            visual = '<visual name=' + "'" + name + "'" + '>'

            # Catch origin
            pose = str([s for s in include if '<pose>' in s][0])
            pose = pose.replace("<pose>","")
            pose = pose.replace("</pose>","")
            x,y,z,= pose.split()[0:3]
            roll,pitch,yaw = pose.split()[3:6]
            origin = "xyz='" + x + " " + y + " " + z + "' rpy='" + roll + " " + pitch + " " + yaw + " " + "'"

            # Get sdf path
            model_name = str([s for s in include if '<uri>' in s][0])
            model_name = model_name.replace("<uri>model://","")
            model_name = model_name.replace("</uri>","")

            # Open inner sdf file 
            model_path = self.find(model_name,'.')
            sdf_path = model_path + '/model.sdf'

            # Rewrite in urdf form
            sdf = self.sdf_refine(sdf_path)
            include_temp = self.convert_link(sdf)[0]

            include_temp[0][0] = link_name + '\n'
            include_temp[0][1] = ' ' + visual + "\n"
            include_temp[0][2] = '  <origin ' + origin + "/>" + "\n"

            include_urdf.append(include_temp)

        return include_urdf

    # convert </joint> 
    def convert_joint(self):
        
        nodes = self.find_nodes(self.sdf,'joint name=','</joint>','generous','finicky')        
        joint_urdf = []

        for node in nodes: # Convert in every joint statement
            
            statement = self.sdf[int(node[0]) : int(node[1])+1]
            # Catch joint name
            joint = str(statement[0])

            # Unify the joint types into "fixed"
            joint = joint.replace('prismatic','fixed')
            joint = joint.replace('revolute','fixed')
            
            # Catch axis
            try: 
                xyz = str([s for s in statement if '<xyz>' in s][0])
                xyz = xyz.replace("<xyz>","")
                xyz = xyz.replace("</xyz>","")
                xyz = ' <axis xyz=' + "'" + xyz + "'/>\n"

            except:
                xyz = ''
            
            # Catch parent link
            parent = str([s for s in statement if '<parent>' in s][0])
            parent = parent.replace("<parent>","")
            parent = parent.replace("</parent>","")
            # Omit the uri notation of sdf
            if ":" in parent:
                truncate = parent.index(":")
                parent   = parent[:truncate]

            # Catch child link
            child = str([s for s in statement if '<child>' in s][0])
            child = child.replace("<child>","")
            child = child.replace("</child>","")
            # Omit the uri notation of sdf
            if ":" in child:
                truncate = child.index(":")
                child    = child[:truncate]



            limit = ''

            # # Catch limits
            # limit = '\n'
            # if '<limit>' in statement: # mostly in prismatic
            #     limit_up = str([s for s in statement if '<upper>' in s][0])
            #     limit_up = limit_up.replace("<upper>","")
            #     limit_up = limit_up.replace("</upper>","")

            #     limit_low = str([s for s in statement if '<lower>' in s][0])
            #     limit_low = limit_low.replace("<lower>","")
            #     limit_low = limit_low.replace("</lower>","")

            #     limit = "<limit lower='" + limit_low + "' upper='" + limit_up + "' />\n"

            # elif 'revolute' in statement: # mostly in rotational joint. If nothing is specified, keep it continuous
            #     limit = "<limit lower='" + "0" + "' upper='" + "2*pi" + "' />\n"
            # else:
            #     limit = "<limit lower='" + "0" + "' upper='" + "0" + "' />\n"


            # Rewrite in urdf form 
            joint_temp = [
            joint + '\n',
            xyz,
            ' <parent link=' + "'" + parent + "'" + '/>\n',
            ' <child link='  + "'" + child  + "'" + '/>\n',
            limit,
            '</joint>\n\n']

            joint_urdf.append(joint_temp)

        return joint_urdf

    def write_urdf(self):

        lines = []
        # Open the file
        urdf = open(self.urdf_path,'w')

        # Initialize the file
        urdf.truncate(0) 

        # Define xml version
        lines.append('<?xml version="1.0"?>' + '\n')

        # </robot> 
        lines.append(self.convert_robot())

        # </link>
        link_lines = self.convert_link(self.sdf)
        link_lines = self.purge(link_lines)
        for link in link_lines:
            lines.append(link)

        # </include>
        include_lines = self.convert_include()
        include_lines = self.purge(include_lines)
        for include in include_lines:
            lines.append(include)

        # </joint>
        joint_lines = self.convert_joint()
        joint_lines = self.purge(joint_lines)
        for joint in joint_lines:
            lines.append(joint)

        # </robot> end
        lines.append('</robot>')

        urdf.writelines(lines)

        return urdf

if __name__=='__main__':
    a = sdf2urdf()
    a.write_urdf()
