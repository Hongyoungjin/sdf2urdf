import numpy as np

class sdf2urdf:

    def __init__(self):

        self.sdf_path  = 'PATH/TO/SDF/FILE'
        self.urdf_path = 'PATH/TO/URDF/FILE'

        self.sdf = self.sdf_refine()
        self.sdf = self.remove_comments()

    # Filter out meaningless strings
    def sdf_refine(self):

        sdf = open(self.sdf_path,'r')
        sdf = sdf.readlines()
        sdf = [line.strip() for line in sdf]
        sdf = np.array(sdf)

        return sdf

    # Remove comments to prevent adverse convertion
    def remove_comments(self):

        nodes = self.find_nodes('<!--','-->','generous','generous')
        comment_indices = np.zeros(1)

        for node in nodes:
            comment_index = np.arange(int(node[1]-node[0]+1)) + int(node[0])
            comment_indices = np.append(comment_indices,comment_index)
            
        comment_indices = list(comment_indices[1:].astype(int))
        sdf = np.delete(self.sdf,comment_indices)
        return sdf
    
    # Find any element that contains certain keyword
    def find_index(self,keyword_start):

        elements = [s for s in self.sdf if keyword_start in s]
        indices = np.zeros(1)

        for element in elements:
            index = np.where(self.sdf == element)
            indices = np.append(indices,index)

        indices = indices[1:]

        return indices

    # Find where </link name ..> ... </link> or </joint name .. > ... </joint> is
    def find_nodes(self,keyword_start,keyword_end,start_option,end_option):

        nodes = np.zeros((1,2))
        if start_option == 'finicky': # Used when finding <include> statements
            indices_start = np.where(self.sdf == keyword_start)[0]

        if start_option == 'generous': # Used when finding <link> & <joint> statements
            indices_start = self.find_index(keyword_start)

        if end_option == 'finicky': # Used to find link and joint statements
            indices_end   = np.where(self.sdf == keyword_end)[0]

        if end_option == 'generous':  # Used when removing comments
            indices_end   = self.find_index(keyword_end)

        # Group two sequential elements; first is starting index, and second is ending index
        for i in range(len(indices_start)):
            node = np.array([indices_start[i],indices_end[i]]).reshape((1,2))
            nodes = np.append(nodes,node,axis=0)
            
        return nodes[1:]

    # convert </model> to </robot>
    def convert_robot(self):

        model_start = [s for s in self.sdf if 'model name=' in s][0]
        robot_start = model_start.replace("model","robot") 
        
        return np.array(robot_start + '\n').reshape((1,1))
    
    # convert </link> 
    def convert_link(self):
        
        # 1. Get link info from "link" Statement
        nodes_from_link = self.find_nodes('link name=','</link>','generous','finicky')

        # 2. Get link info from "include" Statement
        nodes_from_include = self.find_nodes('<include>','</include>','finicky','finicky')
        
        nodes = np.append(nodes_from_link,nodes_from_include,axis=0)
        
        link_urdf = np.zeros(1)
        

        for node in nodes: # Convert in every link statement
            
            statement = self.sdf[int(node[0]) : int(node[1])+1]
            

            # 'Link' statement has visual line
            try: # Catch visual name
                visual = str([s for s in statement if 'visual name=' in s][0])
                # Catch link name
                link = str(statement[0])
                

            # But 'include' statement does not.
            except: 
                # Define visual name
                name_line = str([s for s in statement if '<name>' in s][0])

                name_line = name_line.replace("<name>","")
                name      = name_line.replace("</name>","")
                visual = '<visual name=' + "'" + name + "'" + '>'

                # Catch link name 
                link   = '<link name=' + "'" + name + "'" + '>'
            
            # Catch 3D model import path
            model_line = str([s for s in statement if '<uri>' in s][0])
            model_line = model_line.replace("<uri>","")
            model_line = model_line[:-6]
            model_path = model_line.replace("model","package")
            model_path = "'" + model_path + "'"

            # Catch scale
            # 'Link' statement has scale info
            try: 
                scale_line = str([s for s in statement if '<scale>' in s][0])
                scale_line = scale_line.replace("<scale>","")
                scale      = scale_line.replace("</scale>","")
                scale      = "'" + ' scale=' + scale  + "'"
            
            # But scale info is optional.
            except:
                scale = ''
                pass

            # Catch origin
            pose_line = str([s for s in statement if '<pose>' in s][0])
            pose_line = pose_line.replace("<pose>","")
            pose      = pose_line.replace("</pose>","")
            x,y,z = pose.split()[0:3]
            r,p,y = pose.split()[3:6]
            xyz = "'" + x + " " + y + " " + z + " " + "'"
            rpy = "'" + r + " " + p + " " + y + " " + "'"

            # Rewrite in urdf form 
            link_temp = np.array([
            link + '\n',
            ' ' + visual + "\n",
            '  <origin ' + xyz + rpy + '/>\n',
            '  <geometry>\n',
            '   <mesh filename =' + model_path + scale + '/>' , '\n'
            '  <geometry>\n',
            ' </visual>\n',
            '</link>\n'])

            link_urdf = np.append(link_urdf,link_temp)
        link_urdf = link_urdf[1:]
        
        return link_urdf

    # convert </joint> 
    def convert_joint(self):
        
        nodes = self.find_nodes('joint name=','</joint>','generous','finicky')        
        joint_urdf = np.zeros(1)

        for node in nodes: # Convert in every joint statement
            
            statement = self.sdf[int(node[0]) : int(node[1])+1]
            # Catch joint name
            joint = statement[0]
            
            # Catch axis
            try: 
                xyz_line = str([s for s in statement if '<xyz>' in s][0])
                xyz_line = xyz_line.replace("<xyz>","")
                xyz      = xyz_line.replace("</xyz>","")
                xyz      = ' <axis xyz=' + "'" + xyz + "'/>\n"

            except:
                xyz = ''
            
            # Catch parent link
            parent_line = str([s for s in statement if '<parent>' in s][0])
            parent_line = parent_line.replace("<parent>","")
            parent      = parent_line.replace("</parent>","")
            parent      = parent.replace('::link','')

            # Catch child link
            child_line = str([s for s in statement if '<child>' in s][0])
            child_line = child_line.replace("<child>","")
            child      = child_line.replace("</child>","")
            child      = child.replace("::link",'')

            # Rewrite in urdf form 
            joint_temp = np.array([
            joint + '\n',
            xyz,
            ' <parent link=' + "'" + parent + "'" + '/>\n',
            ' <child link='  + "'" + child  + "'" + '/>\n',
            '</joint>\n'])

            joint_urdf = np.append(joint_urdf,joint_temp)
        
        joint_urdf = joint_urdf[1:]
       
        return joint_urdf

    def write_urdf(self):

        lines = np.zeros(1).reshape((1,1))

        urdf = open(self.urdf_path,'w')
        # Reset the file
        urdf.truncate(0) 
        # Define xml version
        lines = np.append(lines,np.array('<?xml version="1.0"?>' + '\n').reshape((1,1)),axis=1)
        # </robot> start
        lines = np.append(lines,self.convert_robot().reshape((1,1)),axis=1)
        # </link>
        for i in range(len(self.convert_link())):
            lines = np.append(lines,np.array(self.convert_link()[i]).reshape((1,1)),axis=1)
        # </joint>
        for i in range(len(self.convert_joint())):
            lines = np.append(lines,np.array(self.convert_joint()[i]).reshape((1,1)),axis=1)
        # </robot> end
        lines = np.append(lines,np.array('</robot>').reshape((1,1)),axis=1)

        
        lines = list(lines[0][1:])
        

        urdf.writelines(lines)
        return urdf

if __name__=='__main__':
    a = sdf2urdf()
    a.write_urdf()
